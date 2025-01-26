import rclpy
from rclpy.node import Node
import os
from openai import OpenAI
from openai import AsyncOpenAI


from turtlesim.msg import Pose
import gradio as gr
import json
import random

os.environ["OPENAI_API_KEY"] = [YOUR_OPENAI_API_KEY]

use_functions = [
    {
        "type": "function",
        "function": {
            "name": "get_robot_pose",
            "description": "Returns the current pose of the robot.",
            "parameters": {
                "type": "object",
                "properties": {
                    "robot_name": {
                        "type": "string",
                        "description": "The name of the robot to retrieve the pose for (e.g., 'turtle2')"
                    }
                },
                "required": ["robot_name"]
            }
        }
    }
]


class ChatbotNode(Node):
    def __init__(self):
        super().__init__('chatbot_node')
        self.get_logger().info('Chatbot node has been started.')

        self.client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
        self.timer = self.create_timer(5.0, self.timer_callback)

        self.subscription = self.create_subscription(
            Pose,
            '/turtle2/pose',
            self.pose_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.chat_history = []
        self.pose = Pose()

    def get_fruit_price(self, fruit_name):
        fruit_prices = {
            '사과': '1000',
            '바나나': '500',
            '오렌지': '750',
            '배': '800'
        }
        if fruit_name in fruit_prices:
            #print(fruit_name + ':' + fruit_prices[fruit_name])
            return fruit_prices[fruit_name]
        return None
    
    def get_robot_pose(self, robot_name):
        print("get_robot_pose")

        robot_poses = {
            'turtle2': Pose(x=self.pose.x, y=self.pose.y, theta=self.pose.theta, linear_velocity=self.pose.linear_velocity, angular_velocity=self.pose.angular_velocity)
        }

        if robot_name in robot_poses:
            return robot_poses[robot_name]
        else:
            return None

    def process_pose_request(self, user_message, chat_history):
        try:
            msgs = [
                {
                    "role": "system",
                    "content": "당신은 현재 로봇 pose 정보를 알려주는 도우미입니다. 제공된 도구를 사용하여 사용자에게 로봇 위치를 알려주세요."
                },
                {
                    "role": "user",
                    "content": user_message  # 사용자 입력을 사용
                }
            ]
            if chat_history:
                for user, assistant in chat_history:
                    msgs.append({"role": "user", "content": user})
                    msgs.append({"role": "assistant", "content": assistant})
                    msgs.append({"role": "user", "content": user_message})
            
            response = self.client.chat.completions.create(
                model="gpt-4o-mini",
                messages=msgs,
                tools=use_functions
            )             
            response_msg = response.choices[0].message
            tool_calls = response_msg.tool_calls
            # 함수 호출 결과를 저장할 리스트
            proc_messages = []

            print(tool_calls)
            if tool_calls: # 도구 호출이 있는 경우
                available_functions = {
                    "get_robot_pose": self.get_robot_pose
                }
                msgs.append(response_msg)

                for tool_call in tool_calls:
                    function_name = tool_call.function.name
                    function_to_call = available_functions[function_name]
                    function_args = json.loads(tool_call.function.arguments)
                    function_response = function_to_call(**function_args)
                    print("function_response: ", function_response)
                    proc_messages.append({ # 함수 실행 결과를 메시지에 추가
                        "tool_call_id": tool_call.id,
                        "role": "tool",
                        "name": function_name,
                        "content": str(function_response)
                    })
                # 모든 메시지 합치기     
                print("proc_messages: ", proc_messages)            
                msgs.extend(proc_messages)
                final_response = self.client.chat.completions.create(
                    model="gpt-4o-mini",
                    messages=msgs)
                #print("final_response: ", final_response.choices[0].message.content)
                chat_history.append({"role": "user", "content": user_message})
                chat_history.append({"role": "assistant", "content": final_response.choices[0].message.content})            

                return final_response.choices[0].message.content, chat_history     
            else:
                return "robot pose 정보를 찾을 수 없습니다.", chat_history
            
        except Exception as e:            
            if "insufficient_quota" in str(e):
                return "", [("Error", "오류가 발생했습니다: API 사용 한도를 초과했습니다. 플랜 및 결제 정보를 확인하세요.")]
            return "", [("Error", f"오류가 발생했습니다: {str(e)}")]

    def process_fruit_request(self, user_message, chat_history):
        try:            
           
            msgs = [
                {
                    "role": "system",
                    "content": "당신은 과일 가격을 알려주는 도우미입니다. 제공된 도구를 사용하여 사용자에게 과일 가격을 알려주세요."
                },
                {
                    "role": "user",
                    "content": user_message  # 사용자 입력을 사용
                }
            ]
            if chat_history:
                for user, assistant in chat_history:
                    msgs.append({"role": "user", "content": user})
                    msgs.append({"role": "assistant", "content": assistant})
                    msgs.append({"role": "user", "content": user_message})

            # API 호출
            response = self.client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=msgs,
            tools=use_functions
            )                     
            # API 응답 처리
            #print("response: ", response)
            response_msg = response.choices[0].message
            # print("response_msg.role: ", response_msg.role)
            #print("response_msg.content: ", response_msg.content)
            #print("response_msg: ", response_msg)
            tool_calls = response_msg.tool_calls
            #print("tool_calls: ", tool_calls)
            
            # 함수 호출 결과를 저장할 리스트
            proc_messages = []

            if tool_calls: # 도구 호출이 있는 경우
                available_functions = {
                    "get_fruit_price": self.get_fruit_price
                }
                msgs.append(response_msg)

                for tool_call in tool_calls:
                    function_name = tool_call.function.name
                    function_to_call = available_functions[function_name]
                    function_args = json.loads(tool_call.function.arguments)
                    function_response = function_to_call(**function_args)

                    proc_messages.append({ # 함수 실행 결과를 메시지에 추가
                        "tool_call_id": tool_call.id,
                        "role": "tool",
                        "name": function_name,
                        "content": function_response
                    })
                # 모든 메시지 합치기                 
                msgs.extend(proc_messages)
                final_response = self.client.chat.completions.create(
                    model="gpt-3.5-turbo",
                    messages=msgs)
                #print("final_response: ", final_response.choices[0].message.content)
                chat_history.append({"role": "user", "content": user_message})
                chat_history.append({"role": "assistant", "content": final_response.choices[0].message.content})            

                return final_response.choices[0].message.content, chat_history     
            else:
                return "가격 정보를 찾을 수 없습니다.", chat_history
            

        except Exception as e:
            
            if "insufficient_quota" in str(e):
                return "", [("Error", "오류가 발생했습니다: API 사용 한도를 초과했습니다. 플랜 및 결제 정보를 확인하세요.")]
            return "", [("Error", f"오류가 발생했습니다: {str(e)}")]

    with gr.Blocks() as demo:
                chatbot = gr.Chatbot(label="Chat", type="messages")
                user_textbox = gr.Textbox(label="Input", placeholder="Type a message...")
                clear = gr.Button("Clear chat history")

                user_textbox.submit(process_fruit_request, [user_textbox, chatbot], [user_textbox, chatbot])
                clear.click(lambda: None, None, chatbot, queue=False)
                #demo.launch(share=True)

    def pose_callback(self, msg):
        #self.get_logger().info(f'Received pose: {msg}')
        self.pose.x = msg.x
        self.pose.y = msg.y
        self.pose.theta = msg.theta
        self.pose.linear_velocity = msg.linear_velocity
        self.pose.angular_velocity = msg.angular_velocity

    def timer_callback(self):
        self.get_logger().info('Timer callback triggered.')
        user_question = input("무엇을 도와드릴까요? ")
        result, self.chat_history = self.process_pose_request(user_question, self.chat_history)
        print('result: ', result)


def main(args=None):
    rclpy.init(args=args)
    node = ChatbotNode()       
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()