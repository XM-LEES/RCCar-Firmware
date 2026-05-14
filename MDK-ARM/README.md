# MDK-ARM 目录说明

这个目录只保留 Keil 工程输入文件和启动源码。

## 保留文件

- `WHEELTEC.uvprojx`：Keil 工程文件，定义当前 STM32F407 编译目标、源码列表、包含路径和输出目录。
- `WHEELTEC.uvoptx`：Keil 工程选项文件，保留可共享的目标配置和调试配置。
- `startup_stm32f407xx.s`：STM32F407 启动汇编源码，属于固件源码输入。

## 本地生成物

以下内容由 Keil 或 `fromelf` 在本地生成，已经从版本库移除并写入 `.gitignore`：

- `MDK-ARM/WHEELTEC/`：Keil 编译输出目录，包含 `.o`、`.d`、`.crf`、`.axf`、`.hex`、`.map`、`.htm`、`.dep` 等生成文件。
- `WHEELTEC.bin`：工程 post-build 步骤生成的烧录镜像。
- `MDK-ARM/*.lst`：汇编 listing 文件。
- `*.uvguix.*`：Keil 按用户生成的 GUI 状态文件。

需要固件产物时，由用户切换 Windows/Keil 环境后手动编译和烧录。Ubuntu 开发环境只做代码检查和工程路径核对。
