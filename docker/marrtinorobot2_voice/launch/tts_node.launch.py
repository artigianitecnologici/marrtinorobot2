# tts_node_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Crea un nodo per avviare il tuo TTSNode
    tts_node = Node(
        package='marrtinorobot2_voice',
        executable='tts_node.py',
        name='tts_node',
        output='screen'
    )

    # Definisci la descrizione di lancio con il nodo creato
    ld = LaunchDescription()
    ld.add_action(tts_node)

    return ld
