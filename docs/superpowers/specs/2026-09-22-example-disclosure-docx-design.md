# 含实测示例交底书 DOCX 设计说明

## 目标

根据当前 Markdown：

```text
docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md
```

生成对应的可编辑 Word 文档：

```text
docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.docx
```

新文档应完整保留当前 Markdown 的15个技术章节、公式、五段算法伪代码、具体示例、算法增益效果和两张实测结果图片。不得覆盖或修改原完整交底书 DOCX。

## 实现方式

新增专用构建脚本：

```text
docs/patents/tools/build_example_disclosure_docx.py
```

专用脚本复用 `build_disclosure_docx.py` 中已经验证的 Word 样式、A4页面、页脚、可更新目录、目录页码缓存、LibreOffice兼容公式处理和稳定 ZIP 打包功能，但独立定义输入、输出、Pandoc资源目录及当前版本的验证条件。

不修改原生成脚本的默认输入和输出，避免影响原完整交底书生成流程。

## Pandoc转换

Pandoc读取格式采用：

```text
markdown+tex_math_dollars+tex_math_single_backslash
```

转换参数包括：

```text
--to=docx
--toc
--number-sections
--metadata=lang:zh-CN
--resource-path=<Markdown所在目录>
```

设置资源目录是为了将以下相对图片真实嵌入 DOCX：

```text
figures/test-pm-rviz-result.png
figures/test-pm-trajectory-curves.png
```

## Word版式

沿用现有交底书样式：

- A4纵向页面；
- 四边页边距均为1440 twips；
- 正文中文字体为宋体，西文字体为 Times New Roman，小四号；
- 标题和三级标题使用黑体/Arial及现有字号层级；
- 正文首行缩进2字符、1.5倍行距；
- 伪代码使用等线/Courier New；
- 主标题居中；
- 章节采用可见编号；
- 页脚使用居中的动态 PAGE 字段；
- 目录包含15个章节、书签超链接、PAGEREF字段和缓存页码，并保持可更新。

两张结果图片按页面可用宽度等比例缩放，保持原始宽高比，不裁剪，不拉伸；图片说明与图片相邻，避免图片与说明分离到不同页面。

## 公式与符号格式

公式优先由 Pandoc 转换为可编辑 OMML，不把公式整体转成图片。

需要重点验证：

- 质点模型中的点导数、粗体向量和上下标；
- 加速度上下界 (a_r^-<0<a_r^+)；
- 分式、平方、根号和集合区间；
- 切换时刻、缩放系数和分段函数；
- 动态规划递推式；
- 示例中的 (widehat{\boldsymbol q}_i)、(	heta_i)、(M=363) 及 ((M-1)\times0.03=10.86\,\mathrm s)；
- 单位中的上标，例如 (\mathrm{m/s^2})。

对 LibreOffice 6.4 无法正确显示的少量 OMML 结构，允许沿用现有语义等价、仍可编辑的线性文本回退；不得出现红色错误符号、缺失操作数或图片化公式。

## 内容与包验证

生成后验证：

- DOCX ZIP结构完整；
- 文档包含15个有序二级章节；
- “具体示例”和“算法增益效果”标题存在，旧标题不存在；
- 363、10.86 s、0.0226 s及高度范围与 Markdown 一致；
- 两个图片关系和两个 `word/media` 成员存在；
- 两张图片的原始文件哈希与 DOCX 内嵌媒体哈希一致；
- 文档包含可编辑 OMML，且重点公式操作数完整；
- 五段算法伪代码均存在；
- 目录15项均具有超链接和PAGEREF字段；
- A4、页边距、字体、动态页码符合要求。

## 渲染检查

使用 LibreOffice 无界面模式将 DOCX 导出为 PDF，并完成：

- 全页扫描 `¿`、替换字符及其他公式导入错误标记；
- 检查PDF页数为正且页面为A4；
- 渲染并目视检查标题/目录页、公式密集页、伪代码页、具体示例页、RViz图片页、数据曲线图片页和末页；
- 确认公式无截断、中文无乱码、图片不模糊或变形、图注与图片对应。

## 可复现性与隔离

- 连续执行两次专用生成器，输出 DOCX 必须字节一致。
- 原完整交底书 Markdown 和 DOCX 不得被构建脚本读取后写回或覆盖。
- 当前用户对原 DOCX 的修改和 Word 锁文件保持不变且不暂存。
- 不修改 Figure 1 文件或无关目录。
