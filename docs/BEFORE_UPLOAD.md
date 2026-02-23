# Yüklemeden önce son kontrol

GitHub’a zip veya git push ile yüklemeden önce:

## GitHub’da değiştireceğin yerler (README üzerinden)

- **YOUR_USERNAME** → Kendi GitHub kullanıcı adın (README’de 2 yerde: CI badge, clone komutu). Bunu GitHub’da README’yi düzenleyerek yapacağını söyledin.
- İstersen **package.xml** dosyalarındaki `your@email.com` → kendi e‑postan (6 paket: `src/*/package.xml`).

## Zip alırken

- **build/, install/, log/** klasörlerini zip’e **ekleme** (zaten .gitignore’da; bu klasörler varsa zip’e dahil etme).
- Veya projeyi hiç `colcon build` etmediğin bir kopyadan zip’le; o zaman bu klasörler zaten yoktur.

## İsteğe bağlı

- Görseller: `docs/real_robot/robot_photo.jpg`, `docs/images/sim_screenshot.png` ekleyip README’deki tabloyu güncellemek.
- CHANGELOG ve REAL_ROBOT_QUICKSTART içindeki `YOUR_USERNAME` → ilk push sonrası değiştirebilirsin.

## Kontrol edildi (değiştirme)

- LICENSE: Tarik Kahraman.
- README, CONTRIBUTING, SECURITY, hardware/docs, STM32, CI workflow: tutarlı.
- Proje içinde `/home/...` gibi kişisel path yok.
- .gitignore: build, install, log, .cursor, __pycache__, .env vb. var.

Bu dosyayı ilk push sonrası silebilir veya repo’da bırakabilirsin.
