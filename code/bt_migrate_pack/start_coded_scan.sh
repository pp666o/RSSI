#!/bin/bash
set -euo pipefail

# 蓝牙LE Coded PHY 扫描配置脚本
# 功能：复位蓝牙控制器、开启事件掩码、配置PHY、设置并启动扩展扫描

HCI_DEV="${HCI_DEV:-${1:-hci0}}"

sudo_cmd() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  elif [[ "${SUDO_NONINTERACTIVE:-0}" == "1" ]]; then
    sudo -n "$@"
  else
    sudo "$@"
  fi
}

echo "===== 开始执行蓝牙控制器配置 ====="
echo "使用蓝牙控制器: ${HCI_DEV}"

# 1) 复位蓝牙控制器（可选，提升稳定性）
echo "1. 复位蓝牙控制器 ${HCI_DEV}..."
sudo_cmd hcitool -i "${HCI_DEV}" cmd 0x03 0x0003

# 2) 开启 General Event Mask
echo "2. 配置通用事件掩码..."
sudo_cmd hcitool -i "${HCI_DEV}" cmd 0x03 0x0001 ff ff ff ff ff ff ff 3f

# 3) 开启 LE Event Mask（全开）
echo "3. 配置LE事件掩码..."
sudo_cmd hcitool -i "${HCI_DEV}" cmd 0x08 0x0001 ff ff ff ff ff ff ff ff

# 4) 设置默认PHY（Coded PHY）
echo "4. 设置默认Coded PHY..."
sudo_cmd hcitool -i "${HCI_DEV}" cmd 0x08 0x0031 0x00 0x04 0x04

# 5) 设置扩展扫描参数（仅扫描Coded PHY，被动扫描）
echo "5. 配置扩展扫描参数..."
sudo_cmd hcitool -i "${HCI_DEV}" cmd 0x08 0x0041 0x00 0x00 0x04 0x00 0x30 0x00 0x30 0x00

# 6) 使能扩展扫描（持续扫描）
echo "6. 启动扩展扫描..."
sudo_cmd hcitool -i "${HCI_DEV}" cmd 0x08 0x0042 0x01 0x00 0x00 0x00 0x00 0x00

echo "===== 所有蓝牙配置执行完成 ====="
