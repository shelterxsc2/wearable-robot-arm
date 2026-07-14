# scenario-intro-mode 历史快照

> 归档说明：本目录是 INTRO 模式早期独立快照，不是当前构建入口，也不代表根 `src/` 的现状。

INTRO 与 INTERVIEW 后来已经合入根 `src/rga_npu.cpp`，并在 `fourth` 迁移分支上继续加入 FIRST_PERSON、统一控制路由和手势 OSD。因此：

- 日常构建必须使用根目录 `src/`；
- 不要把这里的文件复制回根目录；
- 不要根据这里的旧 README 判断当前 API、编译文件或模式状态；
- 本目录仅用于追溯 INTRO 初版算法和比较历史行为。

当前项目说明见：

- `../README.md`
- `../docs/HANDOFF.md`
- `../docs/MIGRATION.md`
- `../docs/CONTROL.md`

快照的核心历史行为包括：使用 COCO 右腕 index 10 作为介绍手、固定初始空间构图、J5 迟滞和 700 ms 发令限频。当前参数必须以根 `src/rga_npu.cpp` 为准。
