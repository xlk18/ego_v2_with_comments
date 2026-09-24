# LaTeX 专利交底书目录与页眉调整设计

## 目标

调整当前删减版质点轨迹重构算法专利技术交底书的页面结构：删除目录，删除每页顶部显示的章节小标题，同时保留页码并将其置于页面底部居中。

## 修改对象

主文档：

`docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.tex`

重新生成但不提交的构建产物：

`docs/patents/build/point-mass-trajectory-reconstruction-technical-disclosure.pdf`

## 排版方案

- 删除 `\tableofcontents`，使文档不再生成目录页。
- 删除只服务于目录层级的 `\setcounter{tocdepth}{2}`。
- 删除目录后的 `\clearpage`，使正文在标题之后自然开始。
- 在正文开始处采用 `\pagestyle{plain}`，取消默认页面顶部的章节运行标题。
- 使用 `plain` 页面样式默认的页面底部居中页码，不删除页码。
- 不引入 `fancyhdr` 等额外页眉页脚宏包，避免增加与现有 TeX Live 2019 环境不必要的兼容风险。

## 内容边界

- 不恢复此前已经删除的技术章节。
- 不改动正文、公式、伪代码、实测参数和两张结果图片。
- 不改动 Markdown、Word 文档、图片文件及用户其他未提交文件。
- 保持 XeLaTeX、LaTeX Workshop 和 `build/` 输出配置不变。

## 验证

1. 使用现有 `latexmk + XeLaTeX` 配方强制重新编译。
2. 确认生成 PDF 不再包含“目录”页或目录文本。
3. 渲染首页、正文页、公式页、伪代码页和图片页，确认顶部章节小标题消失。
4. 确认各页页码位于底部居中位置。
5. 确认正文、公式、五段伪代码、两张图片和实测参数仍然存在。
6. 检查编译日志不存在致命错误、缺字、溢出或未解析引用。
7. 提交修改并推送至 `origin/main`，同时保持用户原有未提交文件不变。
