#!/usr/bin/env bash
set -euo pipefail

readonly COMMAND_NAME="$(basename -- "$0")"

case "${COMMAND_NAME}" in
  docker)
    if [[ " $* " == *" context inspect "* ]]; then
      exit 0
    fi
    if [[ " $* " == *" image inspect "* ]]; then
      if [[ " $* " == *"{{.Architecture}}"* ]]; then
        echo "arm64"
      else
        echo "fake-image-inspect"
      fi
      exit 0
    fi
    echo "unexpected fake docker invocation: $*" >&2
    exit 1
    ;;
  ssh)
    test -n "${G1_FAKE_LOG:-}" || {
      echo "G1_FAKE_LOG is required" >&2
      exit 1
    }
    readonly REMOTE_COMMAND="${!#}"
    bash -n -c "${REMOTE_COMMAND}"
    printf '%s\n' "$*" >>"${G1_FAKE_LOG}"
    if [[ "${REMOTE_COMMAND}" == *"docker run --rm -i"* && \
          "${REMOTE_COMMAND}" == *"--samples"* ]]; then
      cat >/dev/null
      printf '%s\n' '{"schema_version":1,"probe_type":"g1_realsense_callback_latency","captured_at_epoch_ns":1785847200000000000,"boot_id":"test-boot-id","d435i":{"serial":"346522072810","color":{"width":1920,"height":1080,"fps":15,"format":"bgr8"}},"measurement":{"method":"realsense_global_time_to_host_delivery_proxy","timestamp_domain":"global_time","sample_count":120,"callback_latency_ms":{"min":28.0,"median":32.5,"p95":35.0,"max":38.0},"recommended_img_time_offset_s":-0.0325,"p95_abs_residual_ms":4.5,"exposure_us":{"median":8000.0,"p95":9000.0}},"probe_id":"sha256:c07dcefd97a9a3b61b7fec9771c685b9bf39934f86106cf560ca686bd240ce7f"}'
      exit 0
    fi
    if [[ "${REMOTE_COMMAND}" == *"docker exec -i embodied-unitree-g1 python3 -"* ]]; then
      cat >/dev/null
      if [[ "${G1_FAKE_EMPTY_PROBE_ONCE:-}" == 1 && \
            ! -e "${G1_FAKE_LOG}.probe-once" ]]; then
        : > "${G1_FAKE_LOG}.probe-once"
        exit 0
      fi
      printf '%s\n' '{"boot_id":"test-boot-id","captured_at_epoch_ns":1785847200000000000,"d435i":{"color":{"format":"bgr8","fps":15,"height":1080,"intrinsics":{"coeffs":[0.0,0.0,0.0,0.0,0.0],"distortion_model":"inverse_brown_conrady","fx":1368.246826171875,"fy":1372.2265625,"ppx":981.5875854492188,"ppy":552.4080810546875},"width":1920},"depth_to_color_optical":{"rotation_column_major":[0.9999568462371826,-0.008939534425735474,0.002532962244004011,0.008926372043788433,0.9999468326568604,0.005160734057426453,-0.002578962128609419,-0.005137901287525892,0.9999834895133972],"translation_m":[0.015179511159658432,0.0015000001294538379,0.001500000013038516]},"global_time":[{"enabled":true,"sensor":"Stereo Module"},{"enabled":true,"sensor":"RGB Camera"},{"enabled":true,"sensor":"Motion Module"}],"model":"Intel RealSense D435I","serial":"346522072810"},"mode_machine":4,"mode_pr":0,"network_interface":"eth0","schema_version":1}'
      exit 0
    fi
    if [[ "${REMOTE_COMMAND}" == *"python3 -"* && \
          "${REMOTE_COMMAND}" == *"all_rgb_points.recovered.pcd"* ]]; then
      cat >/dev/null
      if [[ "${REMOTE_COMMAND}" == *"--validate"* ]]; then
        action="validate"
      else
        action="merge"
      fi
      echo "rgb_pcd_recovery=PASS action=${action} sources=34 skipped_zero_filled=10 points=1050000 bytes=16800182 nonzero_rgb=1050000 x_m=[-3.0, 4.0] y_m=[-2.0, 3.0] z_m=[-1.0, 2.0] clean_shutdown=false"
      exit 0
    fi
    if [[ "${REMOTE_COMMAND}" == *"tar -C"* || "${REMOTE_COMMAND}" == *"docker load"* ]]; then
      # Deployment streams archives/images over SSH. Drain the producer so the
      # fake does not close the pipe early and turn a valid test into tar EPIPE.
      cat >/dev/null
    fi
    echo "fake_ssh=ok"
    ;;
  *)
    echo "fake command must be invoked as docker or ssh: ${COMMAND_NAME}" >&2
    exit 1
    ;;
esac
