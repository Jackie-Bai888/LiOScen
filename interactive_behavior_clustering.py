import dashscope
from dashscope import Generation
from dashscope.api_entities.dashscope_response import Role
import pandas as pd
import os
import json
import concurrent.futures
import numpy as np
import re

dashscope.api_key = '***'

SYSTEM_PROMPT = """Role: You are an expert in the field of transportation. You need to cluster the interaction behaviors I input.
Task: Please cluster the interaction behaviors from the perspective of hidden accident patterns. The interaction behaviors are those between the hazard-initiator Vi and the hazard-victim Vv.
The output format: {accident pattern a1:[interaction behavior A, interaction behavior B], accident pattern a2:[interaction behavior C, interaction behavior D]...}
Attentions:The output format is json. An interactive behavior can only be classified into one class, which is the class it is most similar to. No interaction behavior can be omitted, merged, summarized, or paraphrased. Each behavior must be assigned to exactly one accident pattern cluster — the one it is most similar to."""
def extract_information(ibs):
    messages = [
        {'role': Role.SYSTEM, 'content': SYSTEM_PROMPT},
        {'role': Role.USER, 'content': ibs}
    ]
    try:
        response = Generation.call(
            model="qwen-plus",
            messages=messages,
            result_format='message'
        )
        if response.status_code == 200:
            content = response.output.choices[0]['message']['content']
            return content
        else:
            print(f"API请求失败: {response.message}")
            return "error"
    except Exception as e:
        print(f"发生错误: {str(e)}")
        return "error"

def parse_info(raw_str):

    """
    Parse a raw string into a JSON dictionary.

    This function processes a raw string that may contain markdown formatting
    or other non-JSON characters, and attempts to extract a valid JSON object.

    Args:
        raw_str (str): The raw string to be parsed, potentially containing JSON data

    Returns:
        dict: A dictionary parsed from the JSON string

    The function performs the following operations:
    1. Removes surrounding backticks and newline characters
    2. Removes 'json\n' prefix if present
    3. Replaces escaped single quotes with single quotes
    4. Attempts to parse as JSON
    5. If parsing fails, performs additional cleaning:
        - Removes trailing commas before ] and }
        - Extracts the first complete JSON object
        - Attempts parsing again
    """
    json_str = raw_str.strip('` \n')
    json_str = json_str.replace('json\n', '', 1)
    json_str = json_str.replace("\\'", "'")
    try:
        result_dict = json.loads(json_str)
    except json.JSONDecodeError:
        print(json_str)
        json_str = re.sub(r',\s*]', ']', json_str)  # 修复结尾多余逗号
        json_str = re.sub(r',\s*}', '}', json_str)
        # 定位并保留第一个完整的JSON对象
        brace_count = 0
        start_index = -1
        end_index = -1

        for i, char in enumerate(json_str):
            if char == '{':
                if start_index == -1:
                    start_index = i
                brace_count += 1
            elif char == '}':
                brace_count -= 1
                if brace_count == 0 and start_index != -1:
                    end_index = i + 1
                    break

        # 如果找到完整的JSON对象，提取它
        if start_index != -1 and end_index != -1:
            json_str = json_str[start_index:end_index].strip()
        else:
            json_str = json_str.strip()
        result_dict = json.loads(json_str)
    return result_dict
def merge_results(results):
    merged = {}
    for res in results:
        print(type(res))
        if isinstance(res, dict):
            for k, v in res.items():
                merged.setdefault(k, []).extend(v)
    return merged

def main():
    file_path = "*****"
    inte_beh = np.load(file_path).tolist()
    inte_beh = set(inte_beh)
    # 去重但保持顺序
    unique_inte_beh = list(dict.fromkeys(inte_beh))
    # 分块参数配置
    CHUNK_SIZE = 200  # 每批处理100条，根据实际调整
    results = []

    for i in range(0, len(unique_inte_beh), CHUNK_SIZE):
        chunk = unique_inte_beh[i:i + CHUNK_SIZE]
        # 新增数据预处理步骤
        formatted_ibs = "Interaction Behaviors:\n[" + ',\n'.join(
            f'"{ib}"' for ib in chunk  # 为每个行为添加双引号
        ) + "]"
        result = extract_information(formatted_ibs)
        new_result = parse_info(result)
        # results.append(new_result)
        with open('qwen_cluster/3_no_sample_miss2_RQ1.3_cluster_'+str(i)+'_result.json', 'w', encoding='utf-8') as f:
            json.dump(new_result, f, ensure_ascii=False, indent=4)      # print(results)
        print(f"已完成第 {i // CHUNK_SIZE + 1} 批处理")

    # # 合并结果并保存
    # final_result = merge_results(results)
    # with open('qwen_cluster_result.json', 'w', encoding='utf-8') as f:
    #     json.dump(final_result, f, ensure_ascii=False, indent=4)


if __name__ == '__main__':
    main()
