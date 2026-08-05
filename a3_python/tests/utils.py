"""向后兼容 — 所有功能已迁移到 a3_python.fixture_loader

保留此文件作为 re-export，避免破坏可能的旧引用。
新代码请直接 import a3_python.fixture_loader。
"""

from a3_python.fixture_loader import load_fixture_json, targets_from_dict, FIXTURES_DIR  # noqa: F401
