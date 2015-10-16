# Project Report
This is the TeX source and build tools for the project report. Note that some of the styling might be specific to the set of fonts that I have installed on my machine. If you can't compile the document on your machine, you can always remove `\input{style.tex}` from `545Report.tex`. You should be able to compile using just `make`.

The main layout of the report is that each section is its own file in the folder `Sections`. This way, the order in which they are included can be changed from the main `545Report.tex` file. All images are put in the `Images` directory. All filepaths in the report TeX are relative to the location of the `545Report.tex` file, not the location of the TeX source itself.

The styling is right now borrowed from my Lambda Calculus book.