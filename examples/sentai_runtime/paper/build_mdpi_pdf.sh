#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

mkdir -p mdpi_template/figures

# Generic combined markdown with explicit separators between files.
awk 'BEGIN{first=1} FNR==1{if(!first) print "\n"; first=0} {print}' \
  abstract.md introduction.md relatedwork.md Implementation.md \
  implementation_01_system_architecture.md implementation_02_runtime_and_memory.md \
  implementation_03_vision_pipeline.md implementation_04_flight_and_telemetry.md \
  implementation_05_programming_model.md conclusion.md future_work.md > paper_combined.md

# MDPI-oriented body fragments with heading levels shifted into LaTeX sections/subsections.
tail -n +2 introduction.md | pandoc --from=gfm --to=latex > mdpi_template/paper_mdpi_intro.tex
tail -n +2 relatedwork.md | pandoc --from=gfm --to=latex --shift-heading-level-by=-1 > mdpi_template/paper_mdpi_related.tex
tail -n +2 Implementation.md | pandoc --from=gfm --to=latex --shift-heading-level-by=-1 > mdpi_template/paper_mdpi_impl_overview.tex
tail -n +2 implementation_01_system_architecture.md | pandoc --from=gfm --to=latex --shift-heading-level-by=-2 > mdpi_template/paper_mdpi_impl01.tex
tail -n +2 implementation_02_runtime_and_memory.md | pandoc --from=gfm --to=latex --shift-heading-level-by=-2 > mdpi_template/paper_mdpi_impl02.tex
tail -n +2 implementation_03_vision_pipeline.md | pandoc --from=gfm --to=latex --shift-heading-level-by=-2 > mdpi_template/paper_mdpi_impl03.tex
tail -n +2 implementation_04_flight_and_telemetry.md | pandoc --from=gfm --to=latex --shift-heading-level-by=-2 > mdpi_template/paper_mdpi_impl04.tex
tail -n +2 implementation_05_programming_model.md | pandoc --from=gfm --to=latex --shift-heading-level-by=-2 > mdpi_template/paper_mdpi_impl05.tex
tail -n +2 conclusion.md | pandoc --from=gfm --to=latex > mdpi_template/paper_mdpi_conclusion.tex
tail -n +2 future_work.md | pandoc --from=gfm --to=latex > mdpi_template/paper_mdpi_future.tex

# Convert SVG block diagrams into PDF for LaTeX inclusion.
for svg in figures/*.svg; do
  base="$(basename "$svg" .svg)"
  rsvg-convert -f pdf -o "mdpi_template/figures/${base}.pdf" "$svg"
done

# Compile the MDPI-style manuscript.
cd mdpi_template
pdflatex -interaction=nonstopmode sentai_drones.tex >/tmp/sentai_mdpi_pdflatex_1.log || true
pdflatex -interaction=nonstopmode sentai_drones.tex >/tmp/sentai_mdpi_pdflatex_2.log || true
test -f sentai_drones.pdf
cp sentai_drones.pdf ../paper_mdpi_drones.pdf