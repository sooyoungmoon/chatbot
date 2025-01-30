# chatbot_ros2
## Introduction
This repository is for sharing knowledge to -
1) create a chatbot using ([OpenAI API](https://openai.com/index/openai-api/))
2) make the chatbot read/write data from/to ROS2 nodes
3) apply the techniques above to robotics applications

## Where to start?
### Get an OpenAI API Key
You need to get an OpenAI API key for authentication.
- Visit (https://platform.openai.com/docs/overview), create an account, and sign in
- Create a secret key ([settings]-[API Keys]-[+ Create a new secret key])
- Copy the new secret key and store it into you local directory as a file (*.env) - be careful not to reveal the secret key to others!
- You should pay to *add to credit balance* to actually use the OpenAI API Key (please refer to [the pricing policy](https://openai.com/api/pricing/) ) 

### Ways to interact with the OpenAI API
- If you use Python, there is official Python bindings for OpenAI API
- Otherwise, you can interact with the API through
  - HTTP requests from any language
  - official Node.js library  
  - community-maintained library
- In this repository, I'll show you the steps to use OpenAI API with Python bindings

### [A simple example using OpenAI API with Python bindings](https://platform.openai.com/docs/api-reference/streaming) 
```python
from openai import OpenAI

client = OpenAI()

stream = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[{"role": "user", "content": "Say this is a test"}],
    stream=True,
)
for chunk in stream:
    if chunk.choices[0].delta.content is not None:
        print(chunk.choices[0].delta.content, end="")
```

### The steps to use OpenAI API with Python bindings
-First, you need to install the official Python bindings
 ```bash
 pip install openai
 ```




