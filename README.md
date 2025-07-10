# 智能窗户 Intelligent Window System

本项目为基于 STM32 的智能窗户控制系统，支持自动/手动开合窗户，适用于智能家居场景。

## 目录结构

```
智能窗户/
├── applications/         # 应用层代码（主程序、电机控制等）
│   ├── main.c
│   ├── mg90s.c
│   ├── mg90s.h
│   └── ...
├── board/                # 板级支持包（BSP）
│   ├── board.c
│   ├── board.h
│   └── CubeMX_Config/    # STM32CubeMX 工程配置
│       ├── Inc/
│       └── Src/
├── figures/              # 项目相关电路图、结构图
├── Kconfig               # RT-Thread 配置文件
├── SConstruct/SConscript # 构建脚本
├── makefile.targets      # Makefile 目标
├── project.uvprojx       # Keil 工程文件
└── README.md             # 项目说明文档
```

---

如需英文版或有其他补充内容，请告知！  
你可以直接将上面内容复制到你的 `README.md` 文件中。

## 功能简介

- 直流减速电机驱动，实现窗户自动开合
- 可通过主控板按键或外部信号控制
- 适配 RT-Thread 实时操作系统
- 提供 LCD 显示支持

## 快速开始

1. **硬件准备**  
   - STM32F4 系列开发板  
   - 直流减速电机  
   - LCD 显示屏
   - 按键、传感器等外设

2. **软件环境**  
   - [Keil MDK](https://www.keil.com/) 或 [RT-Thread Studio](https://www.rt-thread.io/)
   - [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)（可选）
   - [RT-Thread](https://www.rt-thread.io/)

3. **编译与下载**  
   - 使用 Keil 打开 `project.uvprojx` 工程文件，编译并下载到开发板
   - 或使用 SCons 构建脚本进行命令行编译

4. **运行效果**  
   - 上电后，窗户可根据设定自动或手动开合
   - LCD 显示当前状态（如已连接）

## 主要文件说明

- `applications/main.c`：主程序入口
- `applications/mg90s.c`、`mg90s.h`：电机驱动相关代码（如有需要可重命名为 motor.c/motor.h）
- `board/board.c`、`board.h`：板级初始化
- `CubeMX_Config/`：STM32CubeMX 工程生成文件
- `figures/`：原理图、结构图等

## 贡献

欢迎提交 issue 和 PR，完善本项目！

## License

本项目采用 MIT License，详见 LICENSE 文件。
