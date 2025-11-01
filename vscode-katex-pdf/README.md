Hummingbird KaTeX PDF Export

Export Markdown to PDF using markdown-it with KaTeX (including aligned equations) and support for images referenced relative to the Markdown file (e.g., ../figs paths). Prints with Chromium via Puppeteer.

Features

- markdown-it rendering (no pandoc), with `markdown-it-attrs` and `markdown-it-katex`.
- Preprocesses `\[ ... \]` and `\( ... \)` math to work with KaTeX.
- KaTeX CSS loaded locally from `node_modules` (no network fetches).
- Resolves images using a `<base href>` pointing at the Markdown file directory, so `../figs/...` works.
- Simple PDF settings: page margins and paper format.
- Outputs next to the source as `<name>.katex.pdf`.

Commands

- Export Markdown to PDF (KaTeX): `hummingbird-katex-pdf.exportPdf`

Settings

- `hummingbirdKatexPdf.pageMargins` (string, default `1in`)
- `hummingbirdKatexPdf.paperFormat` (string, default `A4`)

Install / Build / Run

Prerequisites

- Node and npm on PATH: `node -v`, `npm -v`.
- If Puppeteer’s Chromium download is blocked:
  - `setx PUPPETEER_SKIP_DOWNLOAD true`
  - `setx PUPPETEER_EXECUTABLE_PATH "C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe"`
  - Then run `npm install` to use your local Chrome.

Dev host (fastest)

1. `cd vscode-katex-pdf`
2. `npm install`
3. Open the folder `vscode-katex-pdf` in VS Code
4. Run → Start Debugging (F5) to launch the Extension Development Host
5. In the dev host window, open `_hummingbird_sim/Lab3_Review.md`
6. Run the command: “Export Markdown to PDF (KaTeX)”
7. Output: `_hummingbird_sim/Lab3_Review.katex.pdf`

Package VSIX and install

1. `cd vscode-katex-pdf`
2. `npm install`
3. `npx @vscode/vsce package`
4. This produces `vscode-katex-pdf/hummingbird-katex-pdf-<version>.vsix`
5. Install via VS Code: Extensions view → … → Install from VSIX…
   - Or CLI: `code --install-extension vscode-katex-pdf\hummingbird-katex-pdf-0.1.1.vsix`

Notes on ../figs paths

- The exporter injects `<base href="file:///.../">` for the directory containing the Markdown file.
- This allows relative images like `../figs/myplot.png` to resolve during print.

Troubleshooting

- Command not visible: reload VS Code after installing the extension.
- Images not showing: verify the Markdown links are valid and relative to the Markdown file directory. The base tag is injected to help with `../figs/...` paths.
- KaTeX/Math not rendering: ensure display math uses `$$...$$` or `\[ ... \]`. The exporter converts `\[ ... \]` to `$$...$$` before rendering.
- Chromium errors:
  - Set `PUPPETEER_EXECUTABLE_PATH` to your Chrome and `PUPPETEER_SKIP_DOWNLOAD=true` before `npm install`.
  - Example on Windows: `setx PUPPETEER_SKIP_DOWNLOAD true`, `setx PUPPETEER_EXECUTABLE_PATH "C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe"`.

