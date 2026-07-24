#!/bin/bash
xhost +local:docker

CANDIDATES=(glim-cpu glim-gpu)
RUNNING=()

for name in "${CANDIDATES[@]}"; do
  if docker ps --filter "name=^${name}$" --filter "status=running" -q | grep -q .; then
    RUNNING+=("$name")
  fi
done

if [ ${#RUNNING[@]} -eq 0 ]; then
  echo "Нет запущенных контейнеров glim (ни glim-cpu, ни glim-gpu)."
  echo "Запустите один из них: make up-glim GLIM_SERVICE_MODE=cpu|gpu"
  exit 1
elif [ ${#RUNNING[@]} -gt 1 ]; then
  echo "Запущено сразу несколько контейнеров glim: ${RUNNING[*]}"
  echo "Это не должно происходить одновременно — остановите лишний."
  exit 1
fi

echo "Подключаюсь к контейнеру: ${RUNNING[0]}"
docker exec -it "${RUNNING[0]}" bash