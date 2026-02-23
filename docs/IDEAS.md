# Önerilen Eklemeler (Ideas)

Repoyu daha da güçlendirmek için düşünülebilecek eklemeler. Öncelik: yüksek etki / makul çaba.

---

## Hemen eklenebilecekler

- **CI badge** — README’de eklendi. Repo adını push sonrası `YOUR_USERNAME` → kendi kullanıcı adınla değiştir.
- **CHANGELOG.md** — Eklendi (v1.0.0 özeti).
- **.gitattributes** — Eklendi (LF, binary).
- **Media** — README’de “Media” bölümü ve `docs/real_robot/` klasörü (.gitkeep) eklendi; fotoğraf/video linkini sen ekleyebilirsin.
- **Issue / PR şablonları** — Henüz yok; istersen `.github/ISSUE_TEMPLATE/`, `PULL_REQUEST_TEMPLATE.md` eklenebilir.

---

## Kod kalitesi

- **Linter / formatter** — `ruff` veya `black` + `isort` için config; isteğe bağlı `pre-commit` (`.pre-commit-config.yaml`).
- **C/C++** — Zaten `.clang-format` var; CI’da `clang-format --dry-run` veya format check eklenebilir.
- **Stub / mock ile test** — HW bridge için seri port mock’u ile unit test (pyserial’ı mock’layıp komut satırlarını doğrulama).

---

## Dokümantasyon

- **Mimari diyagram (görsel)** — `docs/architecture.svg` veya `.png` (draw.io, Mermaid export); README’de gömülü.
- **API özeti** — Özellikle `mecanum_hw_bridge` ve STM32 `protocol.h` için kısa API notu (zaten docstring’ler var; isteğe bağlı Sphinx).
- **Demo / smoke test** — Gazebo’suz, sadece topic’lerin ve node’ların ayağa kalktığını doğrulayan kısa launch + script (CI’da opsiyonel).

---

## Repo / GitHub

- **Repository description & topics** — GitHub’da kısa açıklama ve etiketler: `ros2`, `mecanum`, `autonomous-robot`, `jetson`, `stm32`, `gazebo`, `nav2`, `slam`.
- **Dependabot** — `.github/dependabot.yml` ile bağımlılık güncelleme önerileri.
- **CITATION.cff** — Akademik referans için (isteğe bağlı).

---

## Özellik (opsiyonel)

- **Docker Compose** — Simülasyon + RViz’i tek komutla ayağa kaldıran `docker-compose.yml`.
- **RQt panel / basit dashboard** — Örn. topic listesi, hız göstergesi (ROS 2 rqt_* paketleriyle).
- **Teleop script** — Klavye veya gamepad ile manuel sürüş (sim veya gerçek robot) için küçük yardımcı script.

---

## Özet öncelik

| Öncelik | Ne | Neden |
|--------|-----|-------|
| 1 | CI badge + CHANGELOG + Media bölümü | Görünürlük, güven, proje hikayesi |
| 2 | Issue/PR template, .gitattributes | Profesyonel repo görüntüsü |
| 3 | Linter config (ruff/black) | Kod tutarlılığı |
| 4 | Mimari görsel, repo topics | Anlaşılırlık ve keşfedilebilirlik |

İstersen bir sonraki adımda 1. önceliktekileri (badge, CHANGELOG, Media placeholder) doğrudan ekleyebilirim.
