//! 构造启发式 + 局部搜索 — W7 实现 (对齐 Python `a3_python/heuristic.py`)
//!
//! W7 计划:
//!   - NN 构造 (载重感知等效距离矩阵, N-start)
//!   - 2-opt / Or-opt 局部搜索 (VND 交替)
//!   - 增量电量校验 (专利创新点 3: 仅重算受影响段, O(k) 而非 O(n))
