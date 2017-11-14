\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{eurobot}[2018/11/14 Custom style for Eurobot report]

\LoadClass{report}

\RequirePackage[utf8]{inputenc}
\RequirePackage[english]{babel}
\RequirePackage[T1]{fontenc}
\RequirePackage{hyperref}

% Use Sans Serif font by default
\renewcommand{\familydefault}{\sfdefault}

% Change margins
\RequirePackage[left=2cm,right=2cm,top=1.5cm,bottom=1.5cm]{geometry}
% Chapter titles
\RequirePackage{titlesec, color, blindtext}

\definecolor{gray75}{gray}{0.75}
\newcommand{\hsp}{\hspace{20pt}}

\titleformat{\chapter}[hang]
	{\Huge\bfseries}
    {\thechapter\hsp\textcolor{gray75}{|}\hsp}
    {0pt}{\Huge\bfseries}
	\titlespacing*{\chapter}{0pt}{30pt}{15pt}
    

% Paragraph margins
\setlength{\parindent}{0em}
\setlength{\parskip}{1em}