# Latex

This directory contains the Latex files for the documentation and reports for the 2018 Eurobot edition.

## Build

The latex files use pygments to perform syntax highlighting through the minted package.
To install pygments, type the following command:

```bash
pip install pygments
```

On windows, you will probably need to open the command prompt or powershell console as administrator.

For the build to succeed, you also need to run Latex with the `--shell-escape` argument so that it can call pygments.