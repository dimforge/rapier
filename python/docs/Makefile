# Minimal Sphinx Makefile. The canonical build entry point is
# `../scripts/build_docs.sh`, which handles cdylib rebuilds and
# deps install. This shim is here so `make -C docs html` still works.

html:
	sphinx-build -b html . _build/html

clean:
	rm -rf _build

.PHONY: html clean
