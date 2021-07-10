LAB=Zumy-2
IMG=img/auto_connect.png img/man_joy_override.png

$(LAB).pdf: $(LAB).tex $(IMG)
	pdflatex $(LAB).tex && pdflatex $(LAB).tex

zip:
	cd resources && zip -r ARTag_Resources.zip ar_tag

clean:
	rm -rf *.pdf *.aux *.log *.gz *.out *.toc *.bbl *.blg
