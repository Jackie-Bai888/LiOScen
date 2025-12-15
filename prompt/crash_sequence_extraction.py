import dashscope
from dashscope import Generation
from dashscope.api_entities.dashscope_response import Role
import pandas as pd
import os
import json
import concurrent.futures

dashscope.api_key = '***'

SYSTEM_PROMPT = """Role: You are an expert in the field of transportation. You need to extract the collision information from the input case summary. 
Task: Please extract the road structure, road speed limit, and the accident pattern sequence (APS) from the provided accident report. The APS must include the initial position and speed of the hazard-initiator Vi, the initial position and speed of the hazard-victim Vv, and their interaction behavior.
The output format: {R: road structure; S: road speed limit; APS: {Vi: {initial position and speed of the hazard-victim Vi}; Vv: {initial position and speed of the hazard-victim Vv; IB: interaction behavior between Vi and Vv}}
Attentions: For road structure, we only focus on whether the road is an intersection.The output format for speed is a numerical value in units of km/h. The initial position of a vehicle only includes the lane it is initially in. If the information for a certain field is not obtained, set that field to None. The output format is json. In IB, V1 and V2 should not appear; instead, Vi and Vv should be used. In IB, it should include the individual behaviors of Vi and Vv, as well as their relative positional relationship. Additionally, the IB should be concise, containing only a single sentence.
Example: 
Input Case Summary:{V1, a 2000 Pontiac Montana minivan, made a left turn from a private driveway onto a northbound 5-lane two-way, dry asphalt roadway on a downhill grade. The posted speed limit on this roadway was 80 kmph (50 MPH). V1 entered the roadway by crossing over the two southbound lanes and then entering the third northbound lane, which was a left turn-only lane at a 4-way intersection. The driver of V1 intended to travel straight through the intersection, and so he began to change lanes to the right. He did not see V2, a 1994 Pontiac Grand Am, that was traveling in the second northbound lane. The northbound roadway had curved to the right prior to the private driveway that V1 had exited. As V1 began to change lanes to the right, the front of V1 contacted the left rear of V2 before coming to final rest on the roadway.
The driver of V1 was a 60-year old male who reported that he had been traveling between 1-17 kmph (1-10 mph) prior to the crash. He had no health-related problems, and had taken no medication prior to the crash. He was rested and traveling back home. He was wearing his prescribed lenses that corrected a myopic (nearsighted) condition. He did not sustain any injuries from the crash and refused treatment.
The Critical Precrash Event for the driver of V1 was when he began to travel over the lane line on the right side of the travel lane. The Critical Reason for the Critical Precrash Event was inadequate surveillance (failed to look, looked but did not see). Associated factors coded to the driver of V1 include an illegal use of a left turn lane (cited by police) and an unfamiliarity with the roadway. As per the driver of V1, this was the first time he had driven on this roadway.
The driver of V2 was a 28-year old woman who reported that she had been traveling between 66-80 kmph (41-50 mph) prior to the crash. She had no health-related problems, and had taken no medication prior to the crash. She was rested and on her way home. She does not wear corrective lenses. She sustained minor injuries and was transported to a local trauma facility.
The Critical Precrash Event for the driver of V2 was when the other vehicle encroached into her lane, from an adjacent lane (same direction) over the left lane line. The Critical Reason for the Critical Precrash Event was not coded to the driver of V2 and no associated factors were coded to her.}
Output:{"R": non intersection, "S":80,"APS Format":{"Vi":{"initial position":"third northbound lane that allows only left turns","speed":"1-17"},"Vv":{"initial position":"second northbound lane","speed":"66-80"}, "IB":Vi changed lanes to the right and collided with Vv, which was traveling in the lane.}}"""

def extract_information(danger_context):
    messages = [
        {'role': Role.SYSTEM, 'content': SYSTEM_PROMPT},
        {'role': Role.USER, 'content': danger_context}
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

def parse_extracted_info(extracted_info):
    # 将 extract_information 返回的非标准 JSON 格式字符串解析为字典
    # 使用正则表达式提取键值对
    # 去除字符串中的 'json\n'
    cleaned_string = extracted_info.replace("json\n", "").replace('```','')
    # 使用 split 方法删除 ### 后面的内容
    cleaned_str = cleaned_string.split('###', 1)[0]
    cleaned_str = cleaned_str.split('Explanation', 1)[0]
    # 去除字符串中的多余空格和换行符
    cleaned_str = cleaned_str.strip()
    # 将字符串转换为字典
    try:
        data_dict = json.loads(cleaned_str)
        return data_dict
    except json.JSONDecodeError as e:
        print(f"无法解析: {e}", cleaned_str)

def batch_process_cases(cases):
    with concurrent.futures.ThreadPoolExecutor(max_workers=10) as executor:
        results = list(executor.map(extract_information, cases))
    return results


def main():
    file_path = '***'
    df = pd.read_csv(file_path)
    cases = df['Case Summary'].tolist()

    batch_size = 500
    for i in range(0, len(cases), batch_size):
        start = i
        end = i + batch_size
        batch_cases = cases[start:end]

        # 批量处理
        results = batch_process_cases(batch_cases)

        # 保存结果
        with open(f'inte_beh_extract_case_information/new_extract_crash_info_{start}_{end - 1}.json', 'w',
                  encoding='utf-8') as f:
            json.dump({
                'data': [{
                    'case_id': int(df['Case ID'][start + j]),
                    'case_summary': batch_cases[j],
                    'crash_information': parse_extracted_info(results[j])
                } for j in range(len(batch_cases))],
            }, f, ensure_ascii=False, indent=4)

        print(f"已保存第 {start} 到 {end - 1} 条数据")


if __name__ == '__main__':
    main()
