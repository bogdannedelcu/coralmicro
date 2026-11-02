---
name: reference_edapa_dataset_viewer
description: "EDAPA synthetic drone datasets (people from above, Unreal) served by the LAN 'Data Viewer' at https://dataset.edapa.proiecte.org/ — cookie login, /dvc/* API, 97 datasets / 648 GB (2026-11-02)"
metadata:
  type: reference
  originSessionId: 0213fcbf-a574-4225-a0ea-7da5f47ce784
  modified: 2026-11-02
---

**URL:** https://dataset.edapa.proiecte.org/ (public cert, valid; `dataset.lan` does not resolve from
this host).  Single-page app "Data Viewer" (nginx + Vite bundle); the API is same-origin, NOT under
/api (every unknown path returns the SPA).  Credentials: user `bogdan`, password given by the
operator in chat (not stored here).

**API (cookie session):** `POST /auth/login` JSON `{"username","password"}` → `GET /auth/me`;
`GET /dvc/datasets` (name, md5, size, nfiles, last_modified); `GET /dvc/list?dataset=X[&path=]`;
`GET /dvc/preview?dataset=X&path=file` (text files); `GET /dvc/thumb?dataset=X&path=img&w=N`;
`GET /dvc/stats?dataset=X` (frames, detections, box size classes, weather, camera_profile,
altitude, heatmap — times out >120 s on the 150k-file sets); `GET /dvc/stores` (MinIO primary,
Backblaze B2 fallback); also `/experiments/*`, `/ai/*`, `/s3/preview`.  Use
`curl -c cj.txt -b cj.txt`, never `-k`.

**Content (2026-11-02):** 97 DVC datasets, 648 GB, 1.57 M files.  Synthetic Unreal (UltraDynamicSky)
aerial captures of people: scenes Barn (barn_10k_* up to 81 GB / 150k files), Beach, Prison,
MountainVillage, Bologna, Cairo, Dubrovnik, Paris, Quarry, modern_city_visdrone; plus many smoke /
probe / profile runs.  Layout per capture: `images/`, `clean_images/`, `labels_box/`,
`labels_seg/`, `labels_pose/`, `labels_ignore/`, `segmentation/`, `body_parts/`, `lidar/`,
`sentaigt/`, `eval/`, `rawdump/`, `data.yaml` (YOLO-pose schema, not a split), `dataset_card.md`,
`split.json`, manifests `synth_edapa_*_manifest.jsonl`, `croissant.json`.  barn_10k_v4: 10 002
frames 1280×960, FOV 57.4°, altitude 3.6–56 m (median 29), boxes median 14×26 px, 87 180 boxes,
10 weather presets.  Relevant for SentAI: reference images for the glass-to-detection latency /
colour-vs-detector experiments ("people seen from above").
