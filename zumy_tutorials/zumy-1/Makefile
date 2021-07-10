LAB=Zumy-1
IMG=img/zumy.jpg img/rosgraph.png

$(LAB).pdf: $(LAB).tex $(IMG)
	pdflatex $(LAB).tex && pdflatex $(LAB).tex

zip:
	cd resources && zip -r ARTag_Resources.zip ar_tag

clean:
	rm -rf *.pdf *.aux *.log *.gz *.out *.toc *.bbl *.blg
