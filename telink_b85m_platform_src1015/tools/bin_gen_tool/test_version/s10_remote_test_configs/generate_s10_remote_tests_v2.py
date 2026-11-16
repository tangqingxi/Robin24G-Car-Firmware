#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
S10 遥控器参数组合测试配置生成工具 V2

功能说明:
    本工具用于生成 S10 遥控器（SOP10 封装）的测试配置文件，支持多种测试模式。
    可同时生成 Flash 版本（256字节，烧录地址 0xF000）和 EFuse 版本（32字节，烧录地址 0x00000）的 bin 文件。

支持多种测试模式：
    1. 单参数测试 (single)    - 每次只变化一个参数，其他保持默认值
    2. 双参数组合测试 (pair)  - 两个参数同时变化，测试参数间的影响
    3. 场景组合 (scenario)    - 预定义的实际使用场景配置
    4. 默认配置 (default)     - 生成默认参数配置
    5. 全部模式 (all)         - 生成上述所有测试模式（默认）

生成的文件：
    - Bin 文件: {TEST_TYPE}_{PARAM}_{VALUE}_0x0f000.bin  (Flash 版本)
               {TEST_TYPE}_{PARAM}_{VALUE}_0x00000.bin (EFuse 版本)
    - JSON 文件: 包含完整的参数配置信息
    - MD 文档  : README_{mode}.md 记录所有生成的配置文件

使用方法:
    python3 generate_s10_remote_tests_v2.py --mode single    # 单参数测试
    python3 generate_s10_remote_tests_v2.py --mode pair      # 双参数组合测试
    python3 generate_s10_remote_tests_v2.py --mode scenario  # 场景组合测试
    python3 generate_s10_remote_tests_v2.py --mode default   # 生成默认配置
    python3 generate_s10_remote_tests_v2.py --mode all       # 生成所有测试模式（默认）

高级选项:
    python3 generate_s10_remote_tests_v2.py --mode default --output-dir ./output  # 指定输出目录

参数说明:
    --mode      测试模式: single/pair/scenario/default/all (默认: all)
    --output-dir/-o  输出目录 (默认: 当前目录)
"""

import json
import os
import sys
import argparse
import itertools
from typing import List, Dict, Tuple, Any

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from bin_gen_test import (
    ConfigGenerator, TargetType, FwType,
    SOP10RemotePin, SOP10RemoteShutdownVoltage, SOP10RemoteLowBatteryVoltage,
    SOP10RemoteIdleTimeout, SOP10RemotePairedIndicator
)


# =============================================================================
# 参数定义
# =============================================================================

PARAM_DEFINITIONS = {
    "shutdown_volt": {
        "name": "关机电压",
        "default": SOP10RemoteShutdownVoltage.DEFAULT,
        "options": [
            (SOP10RemoteShutdownVoltage.DEFAULT, "DEFAULT (不检测)"),
            (SOP10RemoteShutdownVoltage.VOLT_2_0V, "2.0V"),
            (SOP10RemoteShutdownVoltage.VOLT_2_1V, "2.1V"),
            (SOP10RemoteShutdownVoltage.VOLT_2_2V, "2.2V"),
            (SOP10RemoteShutdownVoltage.OFF, "OFF"),
        ]
    },
    "low_batt_volt": {
        "name": "低电电压",
        "default": SOP10RemoteLowBatteryVoltage.DEFAULT,
        "options": [
            (SOP10RemoteLowBatteryVoltage.DEFAULT, "DEFAULT (不检测)"),
            (SOP10RemoteLowBatteryVoltage.VOLT_2_2V, "2.2V"),
            (SOP10RemoteLowBatteryVoltage.VOLT_2_3V, "2.3V"),
            (SOP10RemoteLowBatteryVoltage.VOLT_2_4V, "2.4V"),
            (SOP10RemoteLowBatteryVoltage.OFF, "OFF"),
        ]
    },
    "sleep_timeout": {
        "name": "休眠超时",
        "default": SOP10RemoteIdleTimeout.DEFAULT,
        "options": [
            (SOP10RemoteIdleTimeout.DEFAULT, "DEFAULT"),
            (SOP10RemoteIdleTimeout.TIMEOUT_5S, "5s"),
            (SOP10RemoteIdleTimeout.TIMEOUT_10S, "10s"),
            (SOP10RemoteIdleTimeout.TIMEOUT_30S, "30s"),
            (SOP10RemoteIdleTimeout.TIMEOUT_60S, "60s"),
        ]
    },
    "paired_indicator": {
        "name": "配对指示灯",
        "options": [
            (SOP10RemotePairedIndicator.DEFAULT, "DEFAULT"),
            (SOP10RemotePairedIndicator.OFF, "OFF"),
            (SOP10RemotePairedIndicator.ALWAYS, "ALWAYS"),
        ]
    },
}


# =============================================================================
# 场景定义
# =============================================================================

SCENARIOS = {
    "all_open_paired_indicatorled_always_on": {
        "name": "全开模式",
        "description": "所有参数都非默认值, 配对指示灯一直亮",
        "params": {
            "shutdown_volt": SOP10RemoteShutdownVoltage.VOLT_2_1V,
            "low_batt_volt": SOP10RemoteLowBatteryVoltage.VOLT_2_3V,
            "sleep_timeout": SOP10RemoteIdleTimeout.TIMEOUT_10S,
            "paired_indicator": SOP10RemotePairedIndicator.ALWAYS,
            "access_code_group": 0,
            "normal_tx_power": 0,
        }
    },
    "all_open_paired_indicatorled_off": {
        "name": "全开模式",
        "description": "所有参数都非默认值, 配对指示灯一直灭",
        "params": {
            "shutdown_volt": SOP10RemoteShutdownVoltage.VOLT_2_1V,
            "low_batt_volt": SOP10RemoteLowBatteryVoltage.VOLT_2_3V,
            "sleep_timeout": SOP10RemoteIdleTimeout.TIMEOUT_10S,
            "paired_indicator": SOP10RemotePairedIndicator.OFF,
            "access_code_group": 0,
            "normal_tx_power": 0,
        }
    },
    "all_min_values": {
        "name": "全部最小值模式",
        "description": "所有参数配置为最小值",
        "params": {
            "shutdown_volt": SOP10RemoteShutdownVoltage.DEFAULT,
            "low_batt_volt": SOP10RemoteLowBatteryVoltage.DEFAULT,
            "sleep_timeout": SOP10RemoteIdleTimeout.DEFAULT,
            "paired_indicator": SOP10RemotePairedIndicator.DEFAULT,
            "access_code_group": 0,
            "normal_tx_power": 0,
        }
    },
    "all_max_values": {
        "name": "全部最大值模式",
        "description": "所有参数配置为最大值",
        "params": {
            "shutdown_volt": SOP10RemoteShutdownVoltage.OFF,
            "low_batt_volt": SOP10RemoteLowBatteryVoltage.OFF,
            "sleep_timeout": SOP10RemoteIdleTimeout.TIMEOUT_60S,
            "paired_indicator": SOP10RemotePairedIndicator.ALWAYS,
            "access_code_group": 15,
            "normal_tx_power": 15,
        }
    },
}


# =============================================================================
# 基础配置
# =============================================================================

def get_base_config() -> Dict:
    """获取基础配置模板"""
    base = {
        "name": "S10_REMOTE",
        "efuse_dev_info": {
            "toy_type": 1  # 遥控器类型 (FW_TYPE_S10_REMOTE)
        },
        "cfg_param": {
            # 默认引脚配置
            "pin0": SOP10RemotePin.KEY_COM,
            "pin1": SOP10RemotePin.KEY_F_B,
            "pin2": SOP10RemotePin.KEY_L_R,
            "pin3": SOP10RemotePin.KEY_K_S,
            "pin4": SOP10RemotePin.KEY_P_LED,
            
            # 默认参数值 (使用 DEFAULT 枚举值，在文档中会显示为 "DEFAULT (默认值是...)")
            "shutdown_volt": SOP10RemoteShutdownVoltage.DEFAULT,
            "low_batt_volt": SOP10RemoteLowBatteryVoltage.DEFAULT,
            "sleep_timeout": SOP10RemoteIdleTimeout.TIMEOUT_10S,
            "paired_indicator": SOP10RemotePairedIndicator.DEFAULT,
            "reserved0": 0,
            "reserved1": 0,
            # 移动到尾部的字段（原先在 efuse_dev_info_t）
            "access_code_group": 0,
            "normal_tx_power": 0,
        }
    }
    
    base["description"] = "S10 遥控器"
    return base


# =============================================================================
# 配置生成器
# =============================================================================

def generate_single_param_tests() -> List[Dict]:
    """生成单参数测试配置"""
    configs = []
    base = get_base_config()
    
    for param_name, param_def in PARAM_DEFINITIONS.items():
        for value, desc in param_def["options"]:
            safe_desc = sanitize_filename(desc)
            config = {
                "name": f"S10_REMOTE_SINGLE_{param_name.upper()}_{safe_desc}",
                "description": f"S10 遥控器 - 单参数测试: {param_def['name']}={desc}",
                "efuse_dev_info": base["efuse_dev_info"].copy(),
                "cfg_param": base["cfg_param"].copy(),
                "test_type": "single",
                "test_params": {param_name: {"value": value, "desc": desc, "name": param_def["name"]}}
            }
            config["cfg_param"][param_name] = value
            configs.append(config)
    
    return configs


def generate_pair_tests(selected_params: List[str] = None) -> List[Dict]:
    """生成双参数组合测试配置"""
    configs = []
    base = get_base_config()
    
    # 获取要组合的参数列表
    if selected_params:
        params = [(p, PARAM_DEFINITIONS[p]) for p in selected_params if p in PARAM_DEFINITIONS]
    else:
        params = list(PARAM_DEFINITIONS.items())
    
    # 生成参数对
    param_pairs = list(itertools.combinations(params, 2))
    
    seen_configs = set()
    
    for (name1, def1), (name2, def2) in param_pairs:
        # 为每对参数生成几个关键组合
        opts1 = def1["options"]
        opts2 = def2["options"]
        
        key_combinations = [
            (0, 0),  # 最小 + 最小
            (0, -1),  # 最小 + 最大
            (-1, 0),  # 最大 + 最小
            (len(opts1)//2, len(opts2)//2),  # 中间 + 中间
        ]
        
        for idx, (i, j) in enumerate(key_combinations):
            val1, desc1_raw = opts1[i][0], opts1[i][1]
            val2, desc2_raw = opts2[j][0], opts2[j][1]
            desc1 = sanitize_filename(desc1_raw)
            desc2 = sanitize_filename(desc2_raw)
            
            config = {
                "name": f"S10_REMOTE_PAIR_{name1.upper()}_{desc1}_{name2.upper()}_{desc2}",
                "description": f"S10 遥控器 - 双参数组合: {def1['name']}={desc1_raw}, {def2['name']}={desc2_raw}",
                "efuse_dev_info": base["efuse_dev_info"].copy(),
                "cfg_param": base["cfg_param"].copy(),
                "test_type": "pair",
                "test_params": {
                    name1: {"value": val1, "desc": desc1_raw, "name": def1["name"]},
                    name2: {"value": val2, "desc": desc2_raw, "name": def2["name"]}
                }
            }
            
            config["cfg_param"][name1] = val1
            config["cfg_param"][name2] = val2
            
            if config["name"] in seen_configs:
                continue
            seen_configs.add(config["name"])
            configs.append(config)
    
    return configs


def generate_scenario_tests() -> List[Dict]:
    """生成场景测试配置"""
    configs = []
    base = get_base_config()
    
    for scenario_id, scenario in SCENARIOS.items():
        config = {
            "name": f"S10_REMOTE_SCENE_{scenario_id.upper()}",
            "description": f"S10 遥控器 - 场景: {scenario['name']} ({scenario['description']})",
            "efuse_dev_info": base["efuse_dev_info"].copy(),
            "cfg_param": base["cfg_param"].copy(),
            "test_type": "scenario",
            "test_params": {}
        }
        
        # 应用场景参数
        for param_name, value in scenario["params"].items():
            config["cfg_param"][param_name] = value
            if param_name in PARAM_DEFINITIONS:
                for v, d in PARAM_DEFINITIONS[param_name]["options"]:
                    if v == value:
                        config["test_params"][param_name] = {
                            "value": value,
                            "desc": d,
                            "name": PARAM_DEFINITIONS[param_name]["name"]
                        }
                        break
        
        configs.append(config)
    
    return configs


def generate_default_config() -> List[Dict]:
    """生成默认配置（所有参数使用默认值）"""
    configs = []
    base = get_base_config()
    
    config = {
        "name": f"S10_REMOTE_DEFAULT",
        "description": f"S10 遥控器 - 默认配置（所有参数使用默认值）",
        "efuse_dev_info": base["efuse_dev_info"].copy(),
        "cfg_param": base["cfg_param"].copy(),
        "test_type": "default",
        "test_params": {}
    }
    
    configs.append(config)
    return configs


# =============================================================================
# 文件生成
# =============================================================================

def sanitize_filename(name: str) -> str:
    """清理文件名，替换特殊字符"""
    return name.replace(".", "_")


def convert_enums(obj):
    """转换枚举为值"""
    from enum import IntEnum
    if isinstance(obj, IntEnum):
        return obj.value
    elif isinstance(obj, list):
        return [convert_enums(i) for i in obj]
    elif isinstance(obj, dict):
        return {k: convert_enums(v) for k, v in obj.items()}
    return obj


def generate_files(configs: List[Dict], output_dir: str, doc_entries: List[Dict]):
    """生成 bin 和 json 文件 (同时生成 Flash 和 EFuse 版本，使用相同的 chip_id)"""
    from bin_gen_test import ConfigGenerator, TargetType, FwType
    
    for config in configs:
        config_name = config["name"]
        generated_files = []
        chip_id = None
        
        # 先生成 Flash 版本
        generator_flash = ConfigGenerator(TargetType.FLASH, FwType.REMOTE)
        generator_flash.set_efuse_dev_info(**config["efuse_dev_info"])
        generator_flash.set_cfg_param(**config["cfg_param"])
        flash_bin_file = generator_flash.generate(config_name)
        flash_final_path = os.path.join(output_dir, os.path.basename(flash_bin_file))
        if os.path.abspath(flash_bin_file) != os.path.abspath(flash_final_path):
            if os.path.exists(flash_bin_file):
                if os.path.exists(flash_final_path):
                    os.remove(flash_final_path)
                os.rename(flash_bin_file, flash_final_path)
        
        generated_files.append(os.path.basename(flash_final_path))
        
        # 读取 chip_id
        with open(flash_final_path, 'rb') as f:
            bin_data = f.read()
            chip_id = list(bin_data[0:6])
        
        # 生成 EFuse 版本（使用相同的 chip_id）
        efuse_dev_info_with_chipid = config["efuse_dev_info"].copy()
        efuse_dev_info_with_chipid["chip_id"] = chip_id
        
        generator_efuse = ConfigGenerator(TargetType.EFUSE, FwType.REMOTE)
        generator_efuse.set_efuse_dev_info(**efuse_dev_info_with_chipid)
        generator_efuse.set_cfg_param(**config["cfg_param"])
        efuse_bin_file = generator_efuse.generate(config_name)
        efuse_final_path = os.path.join(output_dir, os.path.basename(efuse_bin_file))
        if os.path.abspath(efuse_bin_file) != os.path.abspath(efuse_final_path):
            if os.path.exists(efuse_bin_file):
                if os.path.exists(efuse_final_path):
                    os.remove(efuse_final_path)
                os.rename(efuse_bin_file, efuse_final_path)
        
        generated_files.append(os.path.basename(efuse_final_path))
        
        # 保存 JSON (只保存一份，包含 Flash 和 EFuse 两个文件名)
        json_data = convert_enums(config)
        json_data["efuse_dev_info"]["chip_id"] = chip_id
        json_data["efuse_dev_info"]["chip_id_hex"] = "".join(f"{b:02x}" for b in chip_id)
        json_data["bin_files"] = {
            "flash": generated_files[0],
            "efuse": generated_files[1]
        }
        
        json_file = os.path.join(output_dir, f"{config_name}.json")
        with open(json_file, 'w', encoding='utf-8') as f:
            json.dump(json_data, f, indent=2, ensure_ascii=False)
        
        # 记录文档 (使用 Flash 版本作为代表)
        doc_entries.append({
            "filename": generated_files[0],
            "efuse_file": generated_files[1],
            "type": config["test_type"],
            "params": config.get("test_params", {}),
            "cfg_param": config["cfg_param"],
            "description": config["description"]
        })
        
        print(f"  Generated: {generated_files[0]} (Flash) + {generated_files[1]} (EFuse)")


def generate_documentation(doc_entries: List[Dict], output_file: str, mode: str):
    """生成文档"""
    is_default_mode = (mode == "default")
    
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(f"# S10 遥控器测试配置文档 - {mode.upper()} 模式\n\n")
        f.write(f"本目录包含 S10 遥控器 **{mode}** 测试模式的配置文件。\n\n")
        
        # 测试模式说明
        mode_descriptions = {
            "single": "单参数测试 - 每次只变化一个参数，其他保持默认值",
            "pair": "双参数组合测试 - 两个参数同时变化，测试参数间的影响",
            "scenario": "场景测试 - 预定义的实际使用场景配置",
            "default": "默认配置 - 所有参数使用默认值",
            "custom": "自定义组合 - 用户指定的特定参数组合"
        }
        f.write(f"## 测试模式说明\n\n")
        f.write(f"{mode_descriptions.get(mode, '自定义测试')}\n\n")
        
        # 统计
        type_counts = {}
        for entry in doc_entries:
            t = entry["type"]
            type_counts[t] = type_counts.get(t, 0) + 1
        
        f.write("## 配置统计\n\n")
        for t, count in sorted(type_counts.items()):
            f.write(f"- {t}: {count} 个\n")
        f.write(f"- **总计**: {len(doc_entries)} 个配置\n\n")
        
        # 管脚映射说明
        f.write("## 管脚映射说明\n\n")
        f.write("SOP10 封装遥控器引脚功能定义（参考 GPIO_PinTypeDef S10RmotePin）：\n\n")
        f.write("| 引脚索引 | GPIO引脚 | 功能说明 |\n")
        f.write("|----------|----------|----------|\n")
        f.write("| pin0 | PA0 | KEY_COM (按键公共端) |\n")
        f.write("| pin1 | PA6 | KEY_F_B (前进/后退按键) |\n")
        f.write("| pin2 | PB0 | KEY_L_R (左转/右转按键) |\n")
        f.write("| pin3 | PA3 | KEY_K_S (功能按键K/S) |\n")
        f.write("| pin4 | PA5 | KEY_P_LED (配对按键/LED输出) |\n")
        f.write("\n")
        
        f.write("### 引脚功能详细说明\n\n")
        f.write("| 功能名称 | enum值 | 说明 |\n")
        f.write("|----------|--------|------|\n")
        f.write("| KEY_COM | 0 | 按键公共端 (接地参考) |\n")
        f.write("| KEY_F_B | 1 | 前进/后退方向按键输入 |\n")
        f.write("| KEY_L_R | 2 | 左转/右转方向按键输入 |\n")
        f.write("| KEY_K_S | 3 | 功能按键K/S输入 |\n")
        f.write("| KEY_P_LED | 4 | 配对按键输入/LED状态输出 |\n")
        f.write("\n")
        
        # 文件列表
        f.write("## 配置文件列表\n\n")
        f.write("| 文件名 | 测试类型 | 参数设置 | 说明 |\n")
        f.write("|--------|----------|----------|------|\n")
        
        for entry in doc_entries:
            # 格式化参数显示
            params_str = ""
            shown_params = set()  # 跟踪已显示的参数
            
            for pname, pval in entry.get("params", {}).items():
                params_str += f"{pname}={pval.get('desc', '')}, "
                shown_params.add(pname)
            
            # 添加其他关键参数（避免重复）
            cfg_param = entry.get("cfg_param", {})
            
            # Pin 配置
            pin_names = {0: "KEY_COM", 1: "KEY_F_B", 2: "KEY_L_R", 3: "KEY_K_S", 4: "KEY_P_LED"}
            for i in range(5):
                pin_val = cfg_param.get(f"pin{i}", 0)
                if pin_val != 0 and f"pin{i}" not in shown_params:  # 非默认值且未显示过
                    params_str += f"pin{i}={pin_names.get(pin_val, pin_val)}, "
            
            # 电压和超时参数（避免重复）
            if "shutdown_volt" not in shown_params and cfg_param.get("shutdown_volt", 4) != 4:
                shutdown_names = {0: "DEFAULT", 1: "2.0V", 2: "2.1V", 3: "2.2V", 4: "OFF"}
                params_str += f"shutdown_volt={shutdown_names.get(cfg_param['shutdown_volt'], cfg_param['shutdown_volt'])}, "
            
            if "low_batt_volt" not in shown_params and cfg_param.get("low_batt_volt", 4) != 4:
                low_batt_names = {0: "DEFAULT", 1: "2.2V", 2: "2.3V", 3: "2.4V", 4: "OFF"}
                params_str += f"low_batt_volt={low_batt_names.get(cfg_param['low_batt_volt'], cfg_param['low_batt_volt'])}, "
            
            if "sleep_timeout" not in shown_params and cfg_param.get("sleep_timeout", 2) != 2:
                sleep_names = {0: "DEFAULT", 1: "5s", 2: "10s", 3: "30s", 4: "60s"}
                params_str += f"sleep_timeout={sleep_names.get(cfg_param['sleep_timeout'], cfg_param['sleep_timeout'])}, "
            
            if "paired_indicator" not in shown_params and cfg_param.get("paired_indicator", 0) != 0:
                paired_names = {0: "OFF", 1: "OFF", 2: "ALWAYS"}
                params_str += f"paired_indicator={paired_names.get(cfg_param['paired_indicator'], cfg_param['paired_indicator'])}, "
            
            # 访问码组和发射功率
            if cfg_param.get("access_code_group", 0) != 0:
                params_str += f"access_code_group={cfg_param['access_code_group']}, "
            if cfg_param.get("normal_tx_power", 0) != 0:
                params_str += f"normal_tx_power={cfg_param['normal_tx_power']}, "
            
            params_str = params_str.rstrip(", ") if params_str else "默认"
            
            f.write(f"| {entry['filename']} | {entry['type']} | {params_str} | {entry['description']} |\n")
        
        f.write("\n")
        
        # 参数配置说明表格
        f.write("## 参数配置说明\n\n")
        
        # Pin 名称映射 (SOP10RemotePin 枚举值)
        pin_value_to_name = {
            0: "KEY_COM",
            1: "KEY_F_B",
            2: "KEY_L_R",
            3: "KEY_K_S",
            4: "KEY_P_LED"
        }
        
        # 配对指示灯映射
        paired_indicator_names = {0: "OFF", 1: "OFF", 2: "ALWAYS"}
        
        # DEFAULT 值的默认描述映射
        default_value_desc = {
            "shutdown_volt": "DEFAULT (默认值是不检测)",
            "low_batt_volt": "DEFAULT (默认值是不检测)",
            "sleep_timeout": "DEFAULT (默认值是10s)",
            "paired_indicator": "DEFAULT (默认值是OFF)",
        }
        
        # 为每个条目显示完整的参数配置
        for entry in doc_entries:
            cfg_param = entry.get("cfg_param", {})
            test_type = entry.get("type", "")
            params = entry.get("params", {})
            config_name = entry.get("filename", "").replace("_0x0f000.bin", "")
            
            f.write(f"### {config_name}\n\n")
            f.write("| 参数 | 值 | 说明 |\n")
            f.write("|------|-----|------|\n")
            
            # 显示所有 pin 配置
            for i in range(5):
                pin_val = cfg_param.get(f"pin{i}", 0)
                pin_name = pin_value_to_name.get(pin_val, f"VALUE_{pin_val}")
                f.write(f"| pin{i} | {pin_name} | 引脚{i}功能配置 |\n")
            
            # 其他固定配置 (保留位等)
            f.write(f"| reserved0 | {cfg_param.get('reserved0', 0)} | 保留位 |\n")
            f.write(f"| reserved1 | {cfg_param.get('reserved1', 0)} | 保留位 |\n")
            
            # 新增的字段（原先在 efuse_dev_info_t，现在移到 CfgParamUnion 尾部）
            f.write(f"| access_code_group | {cfg_param.get('access_code_group', 0)} | 访问码组 |\n")
            f.write(f"| normal_tx_power | {cfg_param.get('normal_tx_power', 0)} | 正常发射功率 |\n")
            
            # 可测试参数
            if is_default_mode:
                # default 模式：显示所有参数的默认值
                for param_name, param_def in PARAM_DEFINITIONS.items():
                    if param_name in cfg_param:
                        param_val = cfg_param[param_name]
                        desc = "-"
                        for v, d in param_def["options"]:
                            if v == param_val:
                                desc = d
                                if desc == "DEFAULT":
                                    desc = default_value_desc.get(param_name, desc)
                                break
                        f.write(f"| {param_name} | {desc} | {param_def['name']} |\n")
            else:
                # 其他模式：显示所有在 PARAM_DEFINITIONS 中的参数
                for param_name, param_def in PARAM_DEFINITIONS.items():
                    if param_name in cfg_param:
                        param_val = cfg_param[param_name]
                        desc = "-"
                        for v, d in param_def["options"]:
                            if v == param_val:
                                desc = d
                                if desc == "DEFAULT":
                                    desc = default_value_desc.get(param_name, desc)
                                break
                        f.write(f"| {param_name} | {desc} | {param_def['name']} |\n")
            
            f.write("\n")
        
        # 参数说明
        f.write("## 可测试参数说明\n\n")
        for param_name, param_def in PARAM_DEFINITIONS.items():
            f.write(f"### {param_name} - {param_def['name']}\n")
            opts = [f"{v}({d})" for v, d in param_def["options"]]
            f.write(f"可选值: {', '.join(opts)}\n\n")
        
        f.write("## 烧录说明\n\n")
        f.write("- **烧录地址**: 0xF000 (Flash)\n")
        f.write("- **烧录地址**: 0x00000 (EFuse)\n")
        f.write("- **文件大小**: 256 bytes (Flash), 32 bytes (EFuse)\n")
        f.write("- **未写数据**: 0xFF\n\n")
    
    print(f"\nDocumentation generated: {output_file}")


# =============================================================================
# 主函数
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='S10 遥控器参数组合测试配置生成工具 V2',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例:
  python %(prog)s --mode single                    # 单参数测试
  python %(prog)s --mode pair                      # 双参数组合
  python %(prog)s --mode scenario                  # 场景测试
  python %(prog)s --mode default                   # 生成默认配置
  python %(prog)s --mode all                       # 生成所有模式
        '''
    )
    
    parser.add_argument('--mode', choices=['single', 'pair', 'scenario', 'default', 'all'], 
                       default='all',
                       help='测试模式 (默认: all)')
    parser.add_argument('--output-dir', '-o', default='.',
                       help='输出目录 (默认: 当前目录)')
    
    args = parser.parse_args()
    
    output_dir = args.output_dir
    os.makedirs(output_dir, exist_ok=True)
    
    print("=" * 70)
    print("S10 遥控器参数组合测试配置生成工具 V2")
    print("=" * 70)
    print(f"测试模式: {args.mode}")
    print(f"输出目录: {output_dir}")
    print()
    
    all_configs = []
    
    if args.mode in ['single', 'all']:
        print("生成单参数测试...")
        configs = generate_single_param_tests()
        all_configs.extend(configs)
        print(f"  -> {len(configs)} 个配置")
    
    if args.mode in ['pair', 'all']:
        print("生成双参数组合测试...")
        configs = generate_pair_tests()
        all_configs.extend(configs)
        print(f"  -> {len(configs)} 个配置")
    
    if args.mode in ['scenario', 'all']:
        print("生成场景测试...")
        configs = generate_scenario_tests()
        all_configs.extend(configs)
        print(f"  -> {len(configs)} 个配置")
    
    if args.mode in ['default']:
        print("生成默认配置...")
        configs = generate_default_config()
        all_configs.extend(configs)
        print(f"  -> {len(configs)} 个配置")
    
    print(f"\n总共生成 {len(all_configs)} 个测试配置")
    print()
    
    # 生成文件
    doc_entries = []
    generate_files(all_configs, output_dir, doc_entries)
    
    # 生成文档
    doc_file = os.path.join(output_dir, f"README_{args.mode}.md")
    generate_documentation(doc_entries, doc_file, args.mode)
    
    print()
    print("=" * 70)
    print("生成完成!")
    print("=" * 70)
    print(f"\n输出文件统计:")
    print(f"  - Bin 文件: {len([f for f in os.listdir(output_dir) if f.endswith('.bin')])} 个")
    print(f"  - JSON 文件: {len([f for f in os.listdir(output_dir) if f.endswith('.json')])} 个")
    print(f"  - 文档: {os.path.basename(doc_file)}")


if __name__ == '__main__':
    main()
