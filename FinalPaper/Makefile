# Makefile for building finalpaper.tex

PDF = finalpaper
TEX = $(PDF).tex

all:
	pdflatex $(TEX)
	bibtex $(PDF)
	pdflatex $(TEX)
	pdflatex $(TEX)
	xdg-open $(PDF).pdf

clean:
	rm -f *.aux *.log *.bbl *.blg *.toc *.out *.synctex.gz
