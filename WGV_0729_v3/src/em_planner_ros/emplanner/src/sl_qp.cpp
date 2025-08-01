#include "../include/sl_qp.h"

using namespace std;

//输入接口: upperbound, lowerbound
//输出数据: finalPath_x, finalPath_y
PiecewiseJerkPathProblem::PiecewiseJerkPathProblem(
    const std::vector<double> ub, const std::vector<double> lb,
    const double delta_s,  //构造函数初始化
    const std::array<double, 3>& x_init) {
  num_of_knots_ = ub.size();

  //起点初始化
  x_init_ = x_init;

  // s点的间隔
  delta_s_ = delta_s;

  //边界初始化
  weight_x_ = 0.0;
  weight_dx_ = 75.0;
  weight_ddx_ = 150.0;
  weight_dddx_ = 500;
  x_bounds_.clear();
  dx_bounds_.clear();
  ddx_bounds_.clear();
  for (int i = 0; i < num_of_knots_; ++i) {
    x_bounds_.push_back(std::make_pair(lb.at(i), ub.at(i)));
    dx_bounds_.push_back(std::make_pair(-1, 1));
    ddx_bounds_.push_back(std::make_pair(-0.2, 0.2));
  }
  dddx_bound_ = std::move(std::make_pair(-0.1, 0.1));

  // 参考线
  x_ref_.resize(num_of_knots_, 0.0);
  has_x_ref_ = true;
  weight_x_ref_vec_ = std::vector<double>(num_of_knots_, 50.0);

  //终点初始化
  has_end_state_ref_ = true;
  std::array<double, 3> end_point = {0.0, 0.0, 0.0};
  end_state_ref_ = std::move(end_point);
  weight_end_state_ = {10000.0, 10.0, 10.0};
}

//求解各矩阵获取
OSQPData* PiecewiseJerkPathProblem::FormulateProblem() {
  // calculate kernel
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(&P_data, &P_indices, &P_indptr);

  // calculate affine constraints
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  CalculateAffineConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds,
                            &upper_bounds);

  // calculate offset
  std::vector<c_float> q;
  CalculateOffset(&q);

  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));

  size_t kernel_dim = 3 * num_of_knots_;
  size_t num_affine_constraint = lower_bounds.size();

  data->n = kernel_dim;             // A矩阵行维度
  data->m = num_affine_constraint;  // A矩阵列维度

  // csc_to_triu可以将csc的矩阵转换为上三角矩阵
  data->P = csc_to_triu(csc_matrix(kernel_dim, kernel_dim, P_data.size(),
                                   CopyData(P_data), CopyData(P_indices),
                                   CopyData(P_indptr)));

  data->q = CopyData(q);
  data->A =
      csc_matrix(num_affine_constraint, kernel_dim, A_data.size(),
                 CopyData(A_data), CopyData(A_indices), CopyData(A_indptr));
  data->l = CopyData(lower_bounds);
  data->u = CopyData(upper_bounds);
  return data;
}

bool PiecewiseJerkPathProblem::Optimize(std::vector<TrajectoryPoint>& qp_path) {
  auto now = std::chrono::system_clock::now();
  auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch());

  OSQPData* data = FormulateProblem();

  OSQPSettings* settings = SolverDefaultSettings();
  settings->max_iter = max_iter_;

  OSQPWorkspace* osqp_work = nullptr;
  osqp_work = osqp_setup(data, settings);

  // osqp_setup(&osqp_work, data, settings);
  osqp_solve(osqp_work);

  auto status = osqp_work->info->status_val;
  cout << status << endl;
  // if(status == -3) exit(1);

  if (status < 0 || (status != 1 && status != 2)) {
    cout << "failed optimization status:\t" << osqp_work->info->status << endl;
    osqp_cleanup(osqp_work);
    FreeData(data);
    c_free(settings);
    return false;
  } else if (osqp_work->solution == nullptr) {
    cout << "The solution from OSQP is nullptr" << endl;
    osqp_cleanup(osqp_work);
    FreeData(data);
    c_free(settings);
    return false;
  }

  // extract primal results
  x_.resize(num_of_knots_);
  dx_.resize(num_of_knots_);
  ddx_.resize(num_of_knots_);

  for (size_t i = 0; i < num_of_knots_; ++i) {
    x_.at(i) = osqp_work->solution->x[i] / scale_factor_[0];
    qp_path.at(i).frenet_info.l = x_.at(i);
    dx_.at(i) = osqp_work->solution->x[i + num_of_knots_] / scale_factor_[1];
    qp_path.at(i).frenet_info.l_ds = dx_.at(i);
    ddx_.at(i) =
        osqp_work->solution->x[i + 2 * num_of_knots_] / scale_factor_[2];
    qp_path.at(i).frenet_info.l_dds = ddx_.at(i);
  }

  // Cleanup
  osqp_cleanup(osqp_work);
  FreeData(data);
  c_free(settings);

  auto end_time = std::chrono::system_clock::now();
  auto end_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time.time_since_epoch());
  cout << "qp time cost: " << end_time_ms.count() - now_ms.count() << endl;
  return true;
}

/*******************************************************************************
 *
 * 1、计算 Hessian 矩阵
 *
 * *****************************************************************************/
void PiecewiseJerkPathProblem::CalculateKernel(std::vector<c_float>* P_data,
                                               std::vector<c_int>* P_indices,
                                               std::vector<c_int>* P_indptr) {
  const int n = static_cast<int>(num_of_knots_);  // 待平滑的点数
  const int num_of_variables = 3 * n;             // 决策变量的数目
  const int num_of_nonzeros = num_of_variables + (n - 1);
  std::vector<std::vector<std::pair<c_int, c_float>>> columns(num_of_variables);
  int value_index = 0;

  // x(i)^2 * (w_x + w_x_ref[i]), w_x_ref might be a uniform value for all
  // x(i) or piecewise values for different x(i)
  for (int i = 0; i < n - 1; ++i) {
    columns[i].emplace_back(i, (weight_x_ + weight_x_ref_vec_[i]) /
                                   (scale_factor_[0] * scale_factor_[0]));
    ++value_index;
  }
  // x(n-1)^2 * (w_x + w_x_ref[n-1] + w_end_x)
  // 末态单独处理
  columns[n - 1].emplace_back(
      n - 1, (weight_x_ + weight_x_ref_vec_[n - 1] + weight_end_state_[0]) /
                 (scale_factor_[0] * scale_factor_[0]));
  ++value_index;

  // x(i)' ^2 * w_dx
  for (int i = 0; i < n - 1; ++i) {
    columns[n + i].emplace_back(
        n + i, weight_dx_ / (scale_factor_[1] * scale_factor_[1]));
    ++value_index;
  }
  // x(n-1)' ^2 * (w_dx + w_end_dx)
  columns[2 * n - 1].emplace_back(2 * n - 1,
                                  (weight_dx_ + weight_end_state_[1]) /
                                      (scale_factor_[1] * scale_factor_[1]));
  ++value_index;

  // 在计算 l'' ^2 的权重时， 考虑了  l''' ^2 的权重
  // 其中 l''' = ( l''(i)-l''(i-1) ) / delta_s
  // l'''^2 = l''(i)^2 /delta_s^2   +  l''(i-1)^2 /delta_s^2  -  2 *
  // l''(i)*l''(i-1) /delta_s^2
  auto delta_s_square = delta_s_ * delta_s_;
  // x(i)'' ^2  * (w_ddx + 2 * w_dddx / delta_s^2)
  // 所以每个 l''(i)^2 的权重有两部分组成，一个是w_ddx， 一个是w_dddx
  columns[2 * n].emplace_back(2 * n,
                              (weight_ddx_ + weight_dddx_ / delta_s_square) /
                                  (scale_factor_[2] * scale_factor_[2]));
  ++value_index;
  for (int i = 1; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(
        2 * n + i, (weight_ddx_ + 2.0 * weight_dddx_ / delta_s_square) /
                       (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }
  columns[3 * n - 1].emplace_back(
      3 * n - 1,
      (weight_ddx_ + weight_dddx_ / delta_s_square + weight_end_state_[2]) /
          (scale_factor_[2] * scale_factor_[2]));
  ++value_index;

  // -2 * w_dddx / delta_s^2 * x(i)'' * x(i + 1)''
  // hession矩阵的 右下角这个 n*n的矩阵，除了对角线元素，它左下侧还有一排元素
  /***       |    o                  |
   *         |         o             |
   *         |              o        |
   *         |              o  o     |
   *         |                 o  0  |
   * ***/
  for (int i = 0; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(2 * n + i + 1,
                                    (-2.0 * weight_dddx_ / delta_s_square) /
                                        (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }

  // 转换成csc_matrix的形式
  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    P_indptr->push_back(ind_p);
    for (const auto& row_data_pair : columns[i]) {
      P_data->push_back(row_data_pair.second *
                        2.0);  // P_data来记录 hession矩阵的元素
      P_indices->push_back(
          row_data_pair.first);  // P_indices来记录各个元素的行号
      ++ind_p;
    }
  }
  P_indptr->push_back(ind_p);
}

/*******************************************************************************
 * g矩阵
 * 目标函数中的 (l-l_ref)^2 = l^2 - 2*l_ref*l + l_ref^2
 * l_ref^2是个非负实数，对梯度没贡献可以忽略，g矩阵中就只剩下 -2*l_ref*l
 *
 * *****************************************************************************/
void PiecewiseJerkPathProblem::CalculateOffset(std::vector<c_float>* q) {
  const int n = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * n;
  q->resize(kNumParam, 0.0);

  if (has_x_ref_) {
    // l^2+(l-ref)^2 拆开项中的 -2ref*i
    for (int i = 0; i < n; ++i) {
      q->at(i) += -2.0 * weight_x_ref_vec_.at(i) * x_ref_[i] / scale_factor_[0];
    }
  }

  if (has_end_state_ref_) {
    q->at(n - 1) +=
        -2.0 * weight_end_state_[0] * end_state_ref_[0] / scale_factor_[0];
    q->at(2 * n - 1) +=
        -2.0 * weight_end_state_[1] * end_state_ref_[1] / scale_factor_[1];
    q->at(3 * n - 1) +=
        -2.0 * weight_end_state_[2] * end_state_ref_[2] / scale_factor_[2];
  }
}

/*******************************************************************************
 *
 * 计算约束条件的 A 矩阵  和上下边界 lower_bounds    upper_bounds
 *
 * *****************************************************************************/
void PiecewiseJerkPathProblem::CalculateAffineConstraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  // 3N个    上下界约束
  // 3(N-1)  连续性约束
  // 3个     初始约束
  const int n = static_cast<int>(num_of_knots_);
  const int num_of_variables = 3 * n;  // 决策变量个数
  const int num_of_constraints =
      num_of_variables + 3 * (n - 1) + 3;  // 约束条件的个数
  lower_bounds->resize(num_of_constraints);
  upper_bounds->resize(num_of_constraints);

  std::vector<std::vector<std::pair<c_int, c_float>>> variables(
      num_of_variables);

  int constraint_index = 0;

  // set x, x', x'' bounds
  for (int i = 0; i < num_of_variables; ++i) {
    if (i < n) {
      variables[i].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) =
          x_bounds_[i].first * scale_factor_[0];
      upper_bounds->at(constraint_index) =
          x_bounds_[i].second * scale_factor_[0];
    } else if (i < 2 * n) {
      variables[i].emplace_back(constraint_index, 1.0);

      lower_bounds->at(constraint_index) =
          dx_bounds_[i - n].first * scale_factor_[1];
      upper_bounds->at(constraint_index) =
          dx_bounds_[i - n].second * scale_factor_[1];
    } else {
      variables[i].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) =
          ddx_bounds_[i - 2 * n].first * scale_factor_[2];
      upper_bounds->at(constraint_index) =
          ddx_bounds_[i - 2 * n].second * scale_factor_[2];
    }
    ++constraint_index;
  }

  // x" 加速度约束
  // x(i->i+1)''' = (x(i+1)'' - x(i)'') / delta_s
  for (int i = 0; i + 1 < n; ++i) {
    variables[2 * n + i].emplace_back(constraint_index, -1.0);
    variables[2 * n + i + 1].emplace_back(constraint_index, 1.0);
    lower_bounds->at(constraint_index) =
        dddx_bound_.first * delta_s_ * scale_factor_[2];
    upper_bounds->at(constraint_index) =
        dddx_bound_.second * delta_s_ * scale_factor_[2];
    ++constraint_index;
  }

  // x' 速度连续性约束
  // x(i+1)' - x(i)' -  0.5*delta_s *x(i)'' -  0.5*delta_s *x(i+1)'' = 0
  for (int i = 0; i + 1 < n; ++i) {
    variables[n + i].emplace_back(constraint_index, -1.0 * scale_factor_[2]);
    variables[n + i + 1].emplace_back(constraint_index, 1.0 * scale_factor_[2]);
    variables[2 * n + i].emplace_back(constraint_index,
                                      -0.5 * delta_s_ * scale_factor_[1]);
    variables[2 * n + i + 1].emplace_back(constraint_index,
                                          -0.5 * delta_s_ * scale_factor_[1]);
    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // x 位置连续性约束
  // x(i+1) =  x(i) + delta_s * x(i)' + 1/3* delta_s^2 * x(i)'' + 1/6*
  // delta_s^2
  // * x(i+1)''
  auto delta_s_sq_ = delta_s_ * delta_s_;
  for (int i = 0; i + 1 < n; ++i) {
    variables[i].emplace_back(constraint_index,
                              -1.0 * scale_factor_[1] * scale_factor_[2]);
    variables[i + 1].emplace_back(constraint_index,
                                  1.0 * scale_factor_[1] * scale_factor_[2]);
    variables[n + i].emplace_back(
        constraint_index, -delta_s_ * scale_factor_[0] * scale_factor_[2]);
    variables[2 * n + i].emplace_back(
        constraint_index,
        -delta_s_sq_ / 3.0 * scale_factor_[0] * scale_factor_[1]);
    variables[2 * n + i + 1].emplace_back(
        constraint_index,
        -delta_s_sq_ / 6.0 * scale_factor_[0] * scale_factor_[1]);
    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // 初始状态约束
  // constrain on x_init、x'_init、x"_init
  variables[0].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
  upper_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
  ++constraint_index;

  variables[n].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
  upper_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
  ++constraint_index;

  variables[2 * n].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
  upper_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
  ++constraint_index;

  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    A_indptr->push_back(ind_p);
    for (const auto& variable_nz : variables[i]) {
      A_data->push_back(variable_nz.second);

      A_indices->push_back(variable_nz.first);
      ++ind_p;
    }
  }
  A_indptr->push_back(ind_p);
}

void PiecewiseJerkPathProblem::FreeData(
    OSQPData* data) {  //释放 OSQPData 结构体中动态分配的内存。
  delete[] data->q;
  delete[] data->l;
  delete[] data->u;

  delete[] data->P->i;
  delete[] data->P->p;
  delete[] data->P->x;

  delete[] data->A->i;
  delete[] data->A->p;
  delete[] data->A->x;
}

OSQPSettings* PiecewiseJerkPathProblem::SolverDefaultSettings() {
  // Define Solver default settings
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);
  settings->polish = true;
  settings->warm_start =
      false;  //求解当前问题时，会使用上一时刻的结果作为初始解
  settings->scaled_termination = true;
  return settings;
}
