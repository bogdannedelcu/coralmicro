#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

mkdir -p docx_assets

for svg in figures/*.svg; do
  base="$(basename "$svg" .svg)"
  rsvg-convert -f png -w 1600 -o "docx_assets/${base}.png" "$svg"
done

cat > paper_docx.md <<'EOF'
---
title: "SentAI: An Open-Source Dual-Camera Edge AI Runtime for Vision-Guided Drone Autonomy on Coral Micro"
subtitle: "MDPI Drones DOCX Draft"
bibliography: references.bib
link-citations: true
...

EOF

cat abstract.md >> paper_docx.md
printf '\n\n' >> paper_docx.md
cat introduction.md >> paper_docx.md
printf '\n\n' >> paper_docx.md
cat relatedwork.md >> paper_docx.md
printf '\n\n' >> paper_docx.md
cat Implementation.md >> paper_docx.md
printf '\n\n' >> paper_docx.md
cat implementation_01_system_architecture.md >> paper_docx.md
printf '\n\n![System architecture block diagram](docx_assets/implementation_01_system_architecture.png)\n\n' >> paper_docx.md
cat implementation_02_runtime_and_memory.md >> paper_docx.md
printf '\n\n![Runtime and memory pipeline diagram](docx_assets/implementation_02_runtime_and_memory.png)\n\n' >> paper_docx.md
cat implementation_03_vision_pipeline.md >> paper_docx.md
printf '\n\n![Dual-camera vision pipeline diagram](docx_assets/implementation_03_vision_pipeline.png)\n\n' >> paper_docx.md
cat implementation_04_flight_and_telemetry.md >> paper_docx.md
printf '\n\n![Flight control and telemetry diagram](docx_assets/implementation_04_flight_and_telemetry.png)\n\n' >> paper_docx.md
cat implementation_05_programming_model.md >> paper_docx.md
printf '\n\n![Programming model diagram](docx_assets/implementation_05_programming_model.png)\n\n' >> paper_docx.md
cat conclusion.md >> paper_docx.md
printf '\n\n' >> paper_docx.md
cat future_work.md >> paper_docx.md
printf '\n\n' >> paper_docx.md
cat section_sources.md >> paper_docx.md

pandoc \
  --from=gfm \
  --to=docx \
  --resource-path=. \
  --citeproc \
  -o paper_mdpi_drones.docx \
  paper_docx.md