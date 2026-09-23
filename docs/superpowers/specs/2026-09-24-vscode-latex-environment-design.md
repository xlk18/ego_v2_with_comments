# VS Code–LaTeX 写作环境与专利文档转换设计

## 1. 目标

在保留系统现有 TeX Live 2019 的前提下，搭建一套可长期用于中英文正式论文写作的 VS Code–LaTeX 环境，并将现有质点轨迹重构算法专利技术交底书 Markdown 文档转换为可独立编译的 XeLaTeX 文档。

完成后的环境应支持中文排版、参考文献、交叉引用、目录、代码与伪代码、PDF 内置预览、SyncTeX 正反向搜索、语法检查、格式化和字数统计。

## 2. 实施边界

- 保留 TeX Live 2019，不安装另一套 TeX Live，也不改变系统默认 TeX 发行版。
- 保留现有 VS Code 用户设置，仅合并 LaTeX 相关配置。
- 安装缺失的系统工具和宏包时，优先使用当前系统的软件包管理方式。
- 不覆盖现有 Markdown 和 Word 交底书；LaTeX 文档作为新的并行版本保存。
- 不修改用户已有的无关工作区文件与未提交改动。

## 3. VS Code 扩展

安装以下扩展：

1. `James-Yu.latex-workshop`：负责构建配方、日志解析、PDF 预览、SyncTeX、引用与命令补全。
2. `ltex-plus.vscode-ltex-plus`：负责 LaTeX 文本的拼写和语言检查；如果当前 VS Code 版本无法安装该扩展，则保留 LaTeX Workshop，并明确记录兼容性情况。

同时在当前工作区创建扩展推荐配置，使以后重新打开工程时能够识别所需扩展。

## 4. TeX 工具链

补齐以下组件：

- `latexmk`：统一管理多轮编译、交叉引用与参考文献流程；
- `biber`：支持现代 BibLaTeX 参考文献工作流；
- `bibtex`：保留传统参考文献兼容性；
- `chktex`：执行 LaTeX 静态检查；
- `latexindent`：格式化 LaTeX 源文件；
- `texcount`：统计正文和公式数量；
- CTeX、XeCJK、字体、图形、数学、表格、代码和交叉引用相关宏包。

默认编译链为 `latexmk` 调用 XeLaTeX。构建产物写入文档同级的 `build/` 目录，避免辅助文件污染源文件目录。

## 5. VS Code 配置

用户级配置负责通用体验：

- 保存时自动构建；
- 默认使用 XeLaTeX 的 `latexmk` 配方；
- PDF 在 VS Code 标签页中预览；
- 启用 SyncTeX；
- 配置清理辅助文件、错误日志显示和 LaTeX 格式化工具。

工作区配置负责本项目约束：

- 指定 `build/` 输出目录；
- 推荐所需扩展；
- 将 LaTeX 临时文件加入忽略规则；
- 不改变 C++、ROS、Python 或 Markdown 的现有配置。

对 JSON 配置的修改采用解析、合并和重新验证方式，禁止整体覆盖用户现有设置。

## 6. Markdown 到 LaTeX 的转换

转换对象为：

`docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md`

生成的主文档放在同一目录，文件基本名保持一致，扩展名改为 `.tex`。转换采用 Pandoc 生成结构化初稿，再进行针对性修订，而不是未经检查地直接交付自动转换结果。

LaTeX 文档采用 `ctexart` 文档类和 XeLaTeX 引擎，并满足以下要求：

- 保留全部技术正文与章节层级；
- 数学内容使用原生 LaTeX 公式，不将公式栅格化；
- 保留两张仿真结果图，图片路径相对于 `.tex` 文件；
- 将 Markdown 伪代码块转换为可跨页的等宽文本环境；
- 生成目录、页码和适合技术交底书阅读的页边距；
- 对百分号、下划线、反斜线等特殊字符进行正确转义；
- 保持现有符号含义、变量定义与公式内容不变。

## 7. 验证方法

环境安装完成后执行以下验证：

1. 检查两个 VS Code 扩展均可被命令行识别。
2. 检查 `latexmk`、XeLaTeX、Biber、BibTeX、ChkTeX、Latexindent 和 Texcount 的可执行文件。
3. 使用默认构建配方实际编译专利 `.tex` 文档。
4. 要求编译成功并生成 PDF，不存在缺图、缺字体、未定义控制序列或致命错误。
5. 检查目录、公式、伪代码、两张图片和中文文本是否进入 PDF。
6. 检查日志中的 overfull/underfull box、未解析引用和缺失字符，并修复实质性排版问题。
7. 运行 ChkTeX、Latexindent 检查和 Texcount，确认辅助工具可用。
8. 验证现有 Markdown、Word 文档及无关用户改动未被覆盖。

## 8. 交付物

- 完整的 VS Code–LaTeX 用户及工作区配置；
- 工作区扩展推荐文件；
- 可独立编译的专利 LaTeX 源文件；
- 成功编译的 PDF；
- 必要的构建说明和常用操作说明；
- 环境与文档验证结果。
