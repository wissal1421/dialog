# Dialogflow ROS 2
This package uses the Google Text-To-Speech (TTS) API to send results to Dialogflow, Google's NLP platform.

Further information can be found at the [ROS wiki](http://wiki.ros.org/dialogflow_ros). Reproduced here:

# Installation
There is an install.sh script available in git directory if you wish to use it, however, I will go over the steps one-by-one here. 

Installing this package requires 3 main steps: cloning the dialogflow repo, setting up your Google cloud project, and setting up Dialogflow. 
However, we need to install PortAudio so we can use PyAudio to get mic data.
```bash
sudo apt-get install portaudio19-dev
```

## Cloning The Repo
Install all the requirements using pip by cloning the Github repo and installing all the packages in requirements.txt.

```bash
cd <workspace>/src
git https://github.com/Juancams/dialogflow_ros2.git
cd dialogflow_ros2
pip install -r requirements.txt
```

## Google Cloud and DialogFlow Setup
Follow the instructions [here](https://cloud.google.com/speech/docs/quickstart) for configuring your Google Cloud project and installing the SDK for authentication. You will need a google/gmail account.

Usage of the Google Cloud SDK requires authentication. This means you require an API key and an activated service account to utilize the APIs.
1. Go to [Google Cloud Console](https://console.cloud.google.com/).
2. Create a new project.
3. Go to the [Kick Setup](https://cloud.google.com/dialogflow/es/docs/quick/setup).
4. Enable API.
5. Create [Service Account](https://console.cloud.google.com/projectselector2/iam-admin/serviceaccounts?walkthrough_id=iam--create-service-account-keys&start_index=1&hl=es-419&supportedpurview=project#step_index=1).
6. Create key & download the JSON File. Rename and move it t your HOME as ~/df_api.json.
7. Go to [DialogFlow Console](https://dialogflow.cloud.google.com/).
8. Create new Agent & select the project.
8. Edit `dialogflow_ros2/config/param.yaml` and write down your project id. You can find it in the [DialogFlow Console](https://dialogflow.cloud.google.com/), clicking in the gear icon.
9. Add `export GOOGLE_APPLICATION_CREDENTIALS='/home/<user>/df_api.json'` to your `.bashrc` and change user.

# Usage
Follow the steps below to setup the package properly.

## Configuration
Go into the config directory and change the following parameters in the `params.yaml` file:

* `results_topic`: (Optional) The topic where your results will be published.
* `project_id`: The name of your project for the Google Speech node. This is the name of your Google Cloud project when going through the Google Cloud setup.

## Launching nodes
To start the Dialogflow nodes, run the following command:
```bash
ros2 launch dialogflow_ros2 dialogflow.launch
```

# ROS Nodes

## mic_client
ROS node receives text from the Google Cloud Speech API and publishes it onto `text_topic` (see config/params.yaml). This is used by the _dialogflow\_client_ node.

### Published Topics
`text_topic` ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))
Acquired text from the Google Cloud Speech API.

## dialogflow_client
ROS node that takes text from the _mic\_client_ node and sends it to Dialogflow for parsing.

### Published Topics
`results_topic` ([dialogflow_msgs/DialogflowResult](http://docs.ros.org/api/dialogflow_msgs/html/msg/DialogflowResult.html))
Publishes a message with the actions, parameters (python dictionary), and fulfillment text associated with the detected intent as a _std\_msgs/String_.

