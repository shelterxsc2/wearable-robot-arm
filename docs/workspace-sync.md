# 开发工作区与仓库同步注意事项

## 1. 目录关系说明

本仓库采用**双区结构**管理下位机代码：

```
RoboticArmControlProject/          # 实际开发工作区（Windows本地）
├── Core/                          # CubeMX生成代码
├── Drivers/                       # CMSIS + HAL库
├── RoboticArmControlSDK/          # 用户业务代码
├── MDK-ARM/                       # Keil工程文件（含编译产物）
├── *.ioc                          # CubeMX配置文件
└── ...                            # 数据手册、图片等临时文件

mcu-stm32/                         # 仓库"干净"版本（Git版本控制）
├── Core/                          # 与上面Core/保持同步
├── Drivers/                       # 与上面Drivers/保持同步
├── RoboticArmControlSDK/          # 与上面SDK/保持同步
├── MDK-ARM/RTE/                   # 仅同步RTE配置
└── README.md                      # 下位机文档
```

| 区域 | 用途 | 是否进Git |
|------|------|-----------|
| `RoboticArmControlProject/` | **本地开发工作区**，Keil编译、调试、烧录均在此 | ❌ 不直接进Git |
| `mcu-stm32/` | **仓库版本**，供版本控制、代码审查、CI/CD | ✅ 进Git |

**原则**：在 `RoboticArmControlProject/` 里开发和编译，定期把**代码源文件**同步到 `mcu-stm32/`，再提交推送。

---

## 2. 需要同步的文件

以下目录/文件**必须**从工作区同步到 `mcu-stm32/`：

```
Core/Src/*.c          # main.c 及外设初始化
Core/Inc/*.h          # 头文件
RoboticArmControlSDK/ # 全部用户业务代码（API/Core/Config/Port）
Drivers/              # HAL库和CMSIS（CubeMX重新生成后需同步）
MDK-ARM/RTE/          # Keil RTE组件配置
```

---

## 3. 不要同步的文件（已.gitignore）

以下文件/目录**严禁**提交到仓库：

```
# 编译产物
*.crf, *.o, *.axf, *.hex, *.map, *.lnp, *.dep
*.lst, *.sct, *.build_log.htm, *.iex

# Keil用户配置（含绝对路径）
*.uvguix.*, *.uvguix.*.bak, *.uvoptx

# 调试配置
.vscode/
DebugConfig/

# 临时文件
*.tmp, *.log

# 数据手册/图片（工作区参考用，不入仓）
*.pdf, *.jpg, *.png
```

---

## 4. 同步方法

### 方式一：手动复制（推荐，可控）

在仓库根目录执行：

```bash
# 同步核心代码
cp -r RoboticArmControlProject/Core          mcu-stm32/
cp -r RoboticArmControlProject/Drivers       mcu-stm32/
cp -r RoboticArmControlProject/RoboticArmControlSDK  mcu-stm32/
cp -r RoboticArmControlProject/MDK-ARM/RTE   mcu-stm32/MDK-ARM/

# 检查变更
git status

# 提交
git add mcu-stm32/
git commit -m "sync: update mcu-stm32 from working dir"
git push origin main
```

### 方式二：脚本同步（批量）

可编写 `scripts/sync_mcu.sh`：

```bash
#!/bin/bash
SRC="RoboticArmControlProject"
DST="mcu-stm32"

cp -r "$SRC/Core"                "$DST/"
cp -r "$SRC/Drivers"             "$DST/"
cp -r "$SRC/RoboticArmControlSDK" "$DST/"
cp -r "$SRC/MDK-ARM/RTE"         "$DST/MDK-ARM/"

echo "Sync done. Run 'git status' to check changes."
```

---

## 5. 常见注意事项

### 5.1 工作区即真相

`RoboticArmControlProject/` 是**唯一可信的编译源**。如果你在 `mcu-stm32/` 里直接改代码，必须手动回拷到工作区，否则下次 Keil 编译会覆盖你的修改。

> **建议**：永远只在 `RoboticArmControlProject/` 里修改，然后把变更同步到 `mcu-stm32/`。

### 5.2 CubeMX重新生成后必须同步

如果你用 STM32CubeMX 重新生成了代码（修改了时钟、引脚、外设等），`Core/` 和 `Drivers/` 会被覆盖。此时**必须**同步到 `mcu-stm32/`，否则仓库里的代码与工程不一致。

### 5.3 避免提交编译产物

Keil 编译后 `MDK-ARM/` 目录下会生成大量 `.crf`、`.o`、`.axf` 等文件。这些已经被 `.gitignore` 排除，但如果你在 `mcu-stm32/MDK-ARM/` 下编译（不建议），请注意不要手动 `git add` 这些文件。

### 5.4 提交前检查

每次提交前执行：

```bash
git diff --cached --stat
```

确认没有混入 `.crf`、`.o`、`.axf` 等编译产物。

---

## 6. 快速检查清单

提交 `mcu-stm32/` 前对照检查：

- [ ] `Core/Src/` 和 `Core/Inc/` 已同步
- [ ] `RoboticArmControlSDK/` 已同步
- [ ] `Drivers/` 如有变更已同步
- [ ] 没有混入 `*.crf`、`*.o`、`*.axf` 等编译产物
- [ ] `git status` 显示的变更都在预期范围内
