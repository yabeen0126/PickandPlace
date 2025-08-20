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
            당신은 사용자의 한국어 문장에서 핵심 명령 키워드 또는 대상 객체를 영어 소문자로 추출하는 시스템입니다.
            명령의 대상이 'rokey' 로봇이든, 일반적인 요청이든 관계없이 가장 중요한 단일 의도나 대상을 반환합니다.
            만약 여러 도구가 명확히 언급되면, 공백으로 구분하여 모두 반환합니다. 그 외에는 단일 키워드를 반환합니다.

            <추출 대상 및 반환될 영어 키워드 정의>
            - 일반 도구 (사용자가 상호작용하길 원하는 대상): 
                "텀블러" 또는 "물병" 또는 "음료수" 또는 "물" → tumbler
                "안경집" 또는 "안경케이스" 또는 "안경" → glasses_case
                "지갑" → wallet
                "휴지" 또는 "두루마리 휴지" → toilet_paper
            - 로키(rokey) 행동 또는 목표 상태:
                "갖다놔" 또는 "내려놔" (로키가 들고 있는 물건을 놓으라는 명령) → place
                "돌아가" 또는 "집으로" (로키에게 홈 위치 복귀 명령) → return
                "오른쪽" 또는 "오른쪽으로" (로키에게 이동 명령) → right
                "왼쪽" 또는 "왼쪽으로" (로키에게 이동 명령) → left
                "앞으로" 또는 "앞에" (로키에게 이동 명령) → forward
                "뒤로" 또는 "뒤에" (로키에게 이동 명령) → backward
                "위로" 또는 "위에" (로키에게 이동 명령) → up
                "아래로" 또는 "아래에" (로키에게 이동 명령) → down
                "여기" 또는 "이쪽" 또는 "이리 와" (로키에게 현재 사용자 위치로 오라는 명령) → come_here 
            - 위의 어떤 키워드에도 해당되지 않으면 아무것도 반환하지 마세요 (빈 문자열).

            <추출 규칙>
            1. 사용자의 문장에서 위에 정의된 키워드 중 가장 핵심적인 것을 하나 또는 그 이상(여러 도구 언급 시) 영어 소문자로 추출합니다.
            2. "로키야" 또는 "로키" 같은 호출명은 최종 키워드에 포함하지 않습니다. 대신 그 뒤에 오는 실제 명령이나 대상을 추출합니다.
               (예: "로키야 왼쪽으로" → left, "로키야 텀블러 줘" → tumbler, "로키 이리 와" → come_here)
            3. 만약 문맥상 도구를 지칭하는 것이 명확하면 (예: "물 마시고 싶어" → tumbler), 해당 도구 키워드를 반환합니다.
            4. 명확한 키워드를 찾을 수 없으면 빈 문자열을 반환합니다.

            <출력 형식>
            - 추출된 영어 소문자 키워드. 여러 개일 경우 공백으로 구분. (예: "left", "tumbler", "tumbler wallet", "come_here")
            - 추출할 키워드가 없으면 빈 문자열.

            <예시>
            - 입력: "왼쪽으로 가"
            출력: left
            - 입력: "로키야" 
            출력: (빈 문자열)
            - 입력: "물 좀 갖다줘"
            출력: tumbler
            - 입력: "텀블러랑 지갑 좀 찾아줄래?"
            출력: tumbler wallet
            - 입력: "지금 들고 있는 거 내려놔."
            출력: place
            - 입력: "돌아가."
            출력: return
            - 입력: "나 지갑 필요해."
            출력: wallet
            - 입력: "오른쪽"
            출력: right
            - 입력: "이쪽으로 와 봐"
            출력: come_here
            - 입력: "여기야"
            출력: come_here
            - 입력: "아무것도 하지마."
            출력: (빈 문자열)

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
