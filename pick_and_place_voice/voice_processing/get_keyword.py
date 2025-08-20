# ros2 service call /get_keyword std_srvs/srv/Trigger "{}"

import os
import rclpy
import pyaudio
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain.chat_models import ChatOpenAI
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain

from std_srvs.srv import Trigger
from voice_processing.MicController import MicController, MicConfig

from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT

############ Package Path & Environment Setting ############
current_dir = os.getcwd()
package_path = get_package_share_directory("pick_and_place_voice")

is_laod = load_dotenv(dotenv_path=os.path.join(f"{package_path}/resource/.env"))
openai_api_key = os.getenv("OPENAI_API_KEY")

############ AI Processor ############
# class AIProcessor:
#     def __init__(self):



############ GetKeyword Node ############
class GetKeyword(Node):
    def __init__(self):


        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
        )

        prompt_content = """
            당신은 사용자의 문장에서 특정 도구와 목적지를 추출해야 합니다.

            <목표>
            - 문장에서 다음 리스트에 포함된 도구를 최대한 정확히 추출하세요.
            - 문장에 등장하는 도구의 목적지(어디로 옮기라고 했는지)도 함께 추출하세요.

            <도구 리스트>
            - hammer, screwdriver, wrench, pos1, pos2, pos3

            <출력 형식>
            - 다음 형식을 반드시 따르세요: [도구1 도구2 ... / pos1 pos2 ...]
            - 도구와 위치는 각각 공백으로 구분
            - 도구가 없으면 앞쪽은 공백 없이 비우고, 목적지가 없으면 '/' 뒤는 공백 없이 비웁니다.
            - 도구와 목적지의 순서는 등장 순서를 따릅니다.

            <특수 규칙>
            - 명확한 도구 명칭이 없지만 문맥상 유추 가능한 경우(예: "못 박는 것" → hammer)는 리스트 내 항목으로 최대한 추론해 반환하세요.
            - 다수의 도구와 목적지가 동시에 등장할 경우 각각에 대해 정확히 매칭하여 순서대로 출력하세요.

            <예시>
            - 입력: "hammer를 pos1에 가져다 놔"  
            출력: hammer / pos1

            - 입력: "왼쪽에 있는 해머와 wrench를 pos1에 넣어줘"  
            출력: hammer wrench / pos1

            - 입력: "왼쪽에 있는 hammer를줘"  
            출력: hammer /

            - 입력: "왼쪽에 있는 못 박을 수 있는것을 줘"  
            출력: hammer /

            - 입력: "hammer는 pos2에 두고 screwdriver는 pos1에 둬"  
            출력: hammer screwdriver / pos2 pos1

            <사용자 입력>
            "{user_input}"                
        """

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = LLMChain(llm=self.llm, prompt=self.prompt_template)
        self.stt = STT(openai_api_key=openai_api_key)


        super().__init__("get_keyword_node")
        # 오디오 설정
        mic_config = MicConfig(
            chunk=12000,
            rate=48000,
            channels=1,
            record_seconds=5,
            fmt=pyaudio.paInt16,
            device_index=10,
            buffer_size=24000,
        )
        self.mic_controller = MicController(config=mic_config)
        # self.ai_processor = AIProcessor()

        self.get_logger().info("MicRecorderNode initialized.")
        self.get_logger().info("wait for client's request...")
        self.get_keyword_srv = self.create_service(
            Trigger, "get_keyword", self.get_keyword
        )
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

    def extract_keyword(self, output_message):
        response = self.lang_chain.invoke({"user_input": output_message})
        result = response["text"]

        object, target = result.strip().split("/")

        object = object.split()
        target = target.split()

        print(f"llm's response: {object}")
        print(f"object: {object}")
        print(f"target: {target}")
        return object
    
    def get_keyword(self, request, response):  # 요청과 응답 객체를 받아야 함
        try:
            print("open stream")
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("Error: Failed to open audio stream")
            self.get_logger().error("please check your device index")
            return None

        while not self.wakeup_word.is_wakeup():
            pass

        # STT --> Keword Extract --> Embedding
        output_message = self.stt.speech2text()
        keyword = self.extract_keyword(output_message)

        self.get_logger().warn(f"Detected tools: {keyword}")

        # 응답 객체 설정
        response.success = True
        response.message = " ".join(keyword)  # 감지된 키워드를 응답 메시지로 반환
        return response


def main():
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
