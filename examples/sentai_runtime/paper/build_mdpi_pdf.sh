#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

mkdir -p mdpi_template/figures
cp references.bib mdpi_template/references.bib

APPENDIX_FILES=(appendices/*.md)

PANDOC_LATEX_ARGS=(--from=markdown+citations --to=latex --natbib --bibliography=references.bib --syntax-highlighting=none)

# Generic combined markdown with explicit separators between files.
awk 'BEGIN{first=1} FNR==1{if(!first) print "\n"; first=0} {print}' \
  abstract.md introduction.md relatedwork.md related_embedded_inference.md \
  experimental_setup.md Implementation.md \
  implementation_01_system_architecture.md implementation_02_runtime_and_memory.md \
  implementation_03_vision_pipeline.md implementation_04_flight_and_telemetry.md \
  implementation_05_programming_model.md \
  memcpy.md cam_switch.md \
  evaluation.md discussion.md \
  threats_to_validity.md statistical_notes.md artifact.md \
  conclusion.md future_work.md glossary.md \
  "${APPENDIX_FILES[@]}" > paper_combined.md

# MDPI-oriented body fragments with heading levels shifted into LaTeX sections/subsections.
tail -n +2 introduction.md | pandoc "${PANDOC_LATEX_ARGS[@]}" > mdpi_template/paper_mdpi_intro.tex
tail -n +2 relatedwork.md | pandoc "${PANDOC_LATEX_ARGS[@]}" --shift-heading-level-by=-1 > mdpi_template/paper_mdpi_related.tex
tail -n +2 Implementation.md | pandoc "${PANDOC_LATEX_ARGS[@]}" --shift-heading-level-by=-1 > mdpi_template/paper_mdpi_impl_overview.tex
tail -n +2 implementation_01_system_architecture.md | pandoc "${PANDOC_LATEX_ARGS[@]}" --shift-heading-level-by=-2 > mdpi_template/paper_mdpi_impl01.tex
tail -n +2 implementation_02_runtime_and_memory.md | pandoc "${PANDOC_LATEX_ARGS[@]}" --shift-heading-level-by=-2 > mdpi_template/paper_mdpi_impl02.tex
tail -n +2 implementation_03_vision_pipeline.md | pandoc "${PANDOC_LATEX_ARGS[@]}" --shift-heading-level-by=-2 > mdpi_template/paper_mdpi_impl03.tex
tail -n +2 implementation_04_flight_and_telemetry.md | pandoc "${PANDOC_LATEX_ARGS[@]}" --shift-heading-level-by=-2 > mdpi_template/paper_mdpi_impl04.tex
tail -n +2 implementation_05_programming_model.md | pandoc "${PANDOC_LATEX_ARGS[@]}" --shift-heading-level-by=-2 > mdpi_template/paper_mdpi_impl05.tex
tail -n +2 discussion.md | pandoc "${PANDOC_LATEX_ARGS[@]}" --shift-heading-level-by=-1 > mdpi_template/paper_mdpi_discussion.tex
tail -n +2 conclusion.md | pandoc "${PANDOC_LATEX_ARGS[@]}" > mdpi_template/paper_mdpi_conclusion.tex
tail -n +2 future_work.md | pandoc "${PANDOC_LATEX_ARGS[@]}" > mdpi_template/paper_mdpi_future.tex

# New chapters added in the experimental-paper expansion.
# Conventions:
#   - related_embedded_inference.md   → \subsection under Related Work
#   - experimental_setup.md           → \section "Experimental Setup"
#   - memcpy.md + cam_switch.md       → \subsection under Results (optimisations)
#   - evaluation.md                   → \section "Results" (cross-cutting)
#   - threats_to_validity.md          → \section "Threats to Validity"
#   - statistical_notes.md            → \subsection "Statistical Notes"
#   - artifact.md                     → \section "Artifact and Data Availability"
#   - glossary.md                     → \section in appendix
tail -n +2 related_embedded_inference.md | pandoc "${PANDOC_LATEX_ARGS[@]}" --shift-heading-level-by=-1 > mdpi_template/paper_mdpi_related_embedded.tex
tail -n +2 experimental_setup.md        | pandoc "${PANDOC_LATEX_ARGS[@]}" --shift-heading-level-by=-1 > mdpi_template/paper_mdpi_expsetup.tex
tail -n +2 memcpy.md                    | pandoc "${PANDOC_LATEX_ARGS[@]}" --shift-heading-level-by=-1 > mdpi_template/paper_mdpi_memcpy.tex
tail -n +2 cam_switch.md                | pandoc "${PANDOC_LATEX_ARGS[@]}" --shift-heading-level-by=-1 > mdpi_template/paper_mdpi_camswitch.tex
tail -n +2 evaluation.md                | pandoc "${PANDOC_LATEX_ARGS[@]}" --shift-heading-level-by=-1 > mdpi_template/paper_mdpi_evaluation.tex
tail -n +2 threats_to_validity.md       | pandoc "${PANDOC_LATEX_ARGS[@]}" --shift-heading-level-by=-1 > mdpi_template/paper_mdpi_threats.tex
tail -n +2 statistical_notes.md         | pandoc "${PANDOC_LATEX_ARGS[@]}" --shift-heading-level-by=-1 > mdpi_template/paper_mdpi_statnotes.tex
tail -n +2 artifact.md                  | pandoc "${PANDOC_LATEX_ARGS[@]}" --shift-heading-level-by=-1 > mdpi_template/paper_mdpi_artifact.tex
tail -n +2 glossary.md                  | pandoc "${PANDOC_LATEX_ARGS[@]}"                              > mdpi_template/paper_mdpi_glossary.tex

awk 'BEGIN{first=1} FNR==1{if(!first) print "\n"; first=0} {print}' "${APPENDIX_FILES[@]}" | pandoc "${PANDOC_LATEX_ARGS[@]}" > mdpi_template/paper_mdpi_appendices.tex

# Convert SVG block diagrams into PDF for LaTeX inclusion.
for svg in figures/*.svg; do
  base="$(basename "$svg" .svg)"
  rsvg-convert -f pdf -o "mdpi_template/figures/${base}.pdf" "$svg"
done

# Mirror the experiment-frame JPEGs into the LaTeX working dir so
# \includegraphics{figures/experiment_frames/...} resolves (pdflatex
# runs inside mdpi_template/, not paper/).
mkdir -p mdpi_template/figures/experiment_frames
cp figures/experiment_frames/*.jpg mdpi_template/figures/experiment_frames/ 2>/dev/null || true

# Compile the MDPI-style manuscript.
cd mdpi_template
pdflatex -interaction=nonstopmode sentai_drones.tex >/tmp/sentai_mdpi_pdflatex_1.log
bibtex sentai_drones >/tmp/sentai_mdpi_bibtex.log
pdflatex -interaction=nonstopmode sentai_drones.tex >/tmp/sentai_mdpi_pdflatex_2.log
pdflatex -interaction=nonstopmode sentai_drones.tex >/tmp/sentai_mdpi_pdflatex_3.log
test -f sentai_drones.pdf
cp sentai_drones.pdf ../paper_mdpi_drones.pdf