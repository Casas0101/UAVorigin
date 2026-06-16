# Reacquisition Workspace

**Status:** clean Python-first mainline  
**Created:** 2026-06-16  
**Planning document:** `../../工程文档/第一环_代码生成总体规划_从零实现_v1.3.md`  
**Execution rules:** `../../工程文档/低智能AI执行规约.md`

This workspace is intentionally clean. It must be populated only by
explicit UXX task cards.

Rules for this workspace:

- implement M0-M6 in Python first;
- do not copy code from `../../ignore_CodeBase/`;
- do not restore the sealed C++ G0 workspace here;
- do not add C++ algorithm files before a human starts the P2 C++ port;
- keep each change scoped to the current UXX task card;
- verify Python work with `python -m pytest -q` unless a task card says
  otherwise.

The old C++ build-failed workspace is outside the active `CodeBase`
tree at:

`../../ignore_CodeBase/`
