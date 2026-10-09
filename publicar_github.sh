#!/usr/bin/env bash
# Uso: ./publicar_github.sh SEU-USUARIO NOME-DO-REPO [public|private]
set -euo pipefail
USUARIO="${1:-}"; REPO="${2:-}"; VIS="${3:-public}"
if [[ -z "$USUARIO" || -z "$REPO" ]]; then
  echo "Uso: $0 SEU-USUARIO NOME-DO-REPO [public|private]"; exit 1
fi
command -v git >/dev/null || { echo "Instale o git."; exit 1; }

[[ -d .git ]] || git init -b main
git add .
git diff --cached --quiet || git commit -m "Corte Gás 10 mm: firmware e CAD"

if command -v gh >/dev/null && gh auth status >/dev/null 2>&1; then
  gh repo create "$USUARIO/$REPO" "--$VIS" --source=. --remote=origin --push
else
  echo "Crie antes o repositório VAZIO em https://github.com/new (nome: $REPO)."
  git remote remove origin 2>/dev/null || true
  git remote add origin "https://github.com/$USUARIO/$REPO.git"
  git push -u origin main
fi
echo "Pronto: https://github.com/$USUARIO/$REPO"
