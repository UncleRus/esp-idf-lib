# Makefile for Sphinx documentation
#

# You can set these variables from the command line.
SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
PAPER         ?=
BUILDDIR      ?= build
SRCDIR        ?= source

# Internal variables.
.DEFAULT_GOAL   = all
ALLSPHINXOPTS   = -d $(BUILDDIR)/doctrees $(PAPEROPT_$(PAPER)) $(SPHINXOPTS)

.PHONY: all help clean html dirhtml singlehtml man

help:
	@echo "Please use \`make <target>' where <target> is one of"
	@echo "  html       to make standalone HTML files"
	@echo "  dirhtml    to make HTML files named index.html in directories"
	@echo "  singlehtml to make a single large HTML file"
	@echo "  man        to make manual pages"
	@echo "  all        to make all of the above"

$(BUILDDIR):
	mkdir $(BUILDDIR)

xml/index.xml: $(BUILDDIR)
	doxygen doxygen.conf
	sed -i 's/LIB8STATIC_ALWAYS_INLINE/inline static/g' xml/*.xml
	sed -i 's/LIB8STATIC/inline static/g' xml/*.xml

clean:
	-rm -rf $(BUILDDIR)/* xml/*

html: xml/index.xml
	$(SPHINXBUILD) -b html $(ALLSPHINXOPTS) $(SRCDIR) $(BUILDDIR)/html
	@echo
	@echo "Build finished. The HTML pages are in $(BUILDDIR)/html."

dirhtml: xml/index.xml
	$(SPHINXBUILD) -b dirhtml $(ALLSPHINXOPTS) $(SRCDIR) $(BUILDDIR)/dirhtml
	@echo
	@echo "Build finished. The HTML pages are in $(BUILDDIR)/dirhtml."

singlehtml: xml/index.xml
	$(SPHINXBUILD) -b singlehtml $(ALLSPHINXOPTS) $(SRCDIR) $(BUILDDIR)/singlehtml
	@echo
	@echo "Build finished. The HTML page is in $(BUILDDIR)/singlehtml."

man: xml/index.xml
	$(SPHINXBUILD) -b man $(ALLSPHINXOPTS) $(SRCDIR) $(BUILDDIR)/man
	@echo
	@echo "Build finished. The manual pages are in $(BUILDDIR)/man."

all: html dirhtml singlehtml man
