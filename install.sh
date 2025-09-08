#!/usr/bin/env bash
set -e

usage() {
  cat <<USAGE
Usage: $0 [--env-file FILE]
Create the Conda environment defined by env.yaml (or another file).

Options:
  --env-file FILE  Path to environment YAML (default: env.yaml)
  -h, --help       Show this help message.
USAGE
}

ENV_FILE="env.yaml"
while [[ $# -gt 0 ]]; do
  case $1 in
    --env-file)
      ENV_FILE="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if ! command -v mamba >/dev/null 2>&1 && ! command -v conda >/dev/null 2>&1; then
  echo "Conda or Mamba is required but not found. Please install Miniconda/Mambaforge." >&2
  exit 1
fi

if command -v mamba >/dev/null 2>&1; then
  CONDA_CMD=mamba
else
  CONDA_CMD=conda
fi

ENV_NAME=$(grep '^name:' "$ENV_FILE" | awk '{print $2}')

# create or update environment
$CONDA_CMD env create -f "$ENV_FILE" 2>/dev/null || $CONDA_CMD env update -f "$ENV_FILE"

echo "Environment '$ENV_NAME' is ready. Activate with:\n  conda activate $ENV_NAME"
