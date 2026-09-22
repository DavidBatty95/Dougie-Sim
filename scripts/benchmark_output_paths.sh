#!/usr/bin/env bash

# Resolve benchmark artifacts onto the writable ros2_ws Docker bind mount.
resolve_benchmark_output_paths() {
  local repo_root="$1"
  local output="$2"
  local result_dir="$3"
  local result_dir_set="$4"
  local default_basename="$5"
  local candidate
  local output_name

  if [ -n "${output}" ] && [ "${result_dir_set}" = "1" ]; then
    printf '%s\n' '--output and --result-dir are mutually exclusive' >&2
    return 2
  fi

  if [ -n "${output}" ]; then
    case "${output}" in
      /workspace/*) candidate="${repo_root}/${output#/workspace/}" ;;
      /*) candidate="${output}" ;;
      *) candidate="${repo_root}/${output}" ;;
    esac
    HOST_JSON="$(realpath -m -- "${candidate}")"
    output_name="$(basename -- "${HOST_JSON}")"
    case "${output_name}" in
      .json)
        printf 'Output filename must have a nonempty stem: %s\n' "${HOST_JSON}" >&2
        return 2
        ;;
      *.json) ;;
      *)
        printf 'Output must name a lowercase .json file: %s\n' "${HOST_JSON}" >&2
        return 2
        ;;
    esac
    RESULT_DIR="$(dirname -- "${HOST_JSON}")"
    BASENAME="${output_name%.json}"
  else
    case "${result_dir}" in
      /workspace/*) candidate="${repo_root}/${result_dir#/workspace/}" ;;
      /*) candidate="${result_dir}" ;;
      *) candidate="${repo_root}/${result_dir}" ;;
    esac
    RESULT_DIR="$(realpath -m -- "${candidate}")"
    BASENAME="${default_basename}"
    HOST_JSON="${RESULT_DIR}/${BASENAME}.json"
  fi

  case "${HOST_JSON}" in
    "${repo_root}/ros2_ws/"*) ;;
    *)
      printf 'Benchmark output must be under %s/ros2_ws: %s\n' \
        "${repo_root}" "${HOST_JSON}" >&2
      return 2
      ;;
  esac

  HOST_STEM="${HOST_JSON%.json}"
  CONTAINER_JSON="/workspace/ros2_ws/${HOST_JSON#${repo_root}/ros2_ws/}"
  CONTAINER_STEM="${CONTAINER_JSON%.json}"
}
