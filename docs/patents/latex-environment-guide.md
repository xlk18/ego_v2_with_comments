# VS Code–LaTeX 使用说明

## 打开与编译

在 VS Code 中打开 `.tex` 文件，保存后由 `latexmk (xelatex)` 自动编译。执行命令面板中的 `LaTeX Workshop: View LaTeX PDF file` 可在标签页中查看 PDF。

命令行等价操作：

```bash
latexmk -xelatex -synctex=1 -interaction=nonstopmode -file-line-error -outdir=build 文件名.tex
```

## 正反向搜索

在 `.tex` 中执行 `SyncTeX from cursor` 跳转到 PDF；在内置 PDF 中按 Ctrl 并单击可返回源文件。

## 参考文献

BibLaTeX 文档使用 Biber，传统 BibTeX 文档使用 BibTeX。`latexmk` 会依据文档配置自动执行所需步骤。

## 格式化、检查与统计

```bash
mkdir -p build
latexindent -c build/ 文件名.tex > /tmp/formatted.tex
chktex -q 文件名.tex
texcount -inc -sum 文件名.tex
```

## 清理构建产物

```bash
latexmk -outdir=build -c 文件名.tex
```

## 常见问题

本环境固定使用系统 TeX Live 2019。若出版社模板要求更新宏包，应先核对模板要求，不要同时安装第二套 TeX Live；确需升级时再整体迁移发行版。
