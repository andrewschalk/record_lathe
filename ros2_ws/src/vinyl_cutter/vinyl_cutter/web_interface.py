from flask import Flask, render_template, request
import os
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Bool
import threading

class WebInterface(Node):
    def __init__(self, host="0.0.0.0", port=5000, upload_folder="audio"):
        super().__init__('web_interface')

        # Publishers
        self.publisher_inner_dia   = self.create_publisher(Int32, 'inner_dia', 10)
        self.publisher_outer_dia   = self.create_publisher(Int32, 'outer_dia', 10)
        self.publisher_cycle_start = self.create_publisher(Int32, 'cycle_start', 10)

        self.host = host
        self.port = port
        self.upload_folder = upload_folder

        # Folder to save audio files
        os.makedirs(self.upload_folder, exist_ok=True)

        # Create Flask app
        self.app = Flask(__name__)

        # Register routes
        self._register_routes()

    def _register_routes(self):
        """Register all of the routes. Call these functions when actions are performed in the web page."""
        @self.app.route("/")
        def index():
            return render_template("index.html")

        @self.app.route("/cycle_start_action", methods=["POST"])
        def cycle_start_action():
            """Publishes "True" to the cycle_start topic when the cycle start button is pressed."""
            msg = Bool()
            msg.data = True
            self.publisher_cycle_start.publish(msg)
            return "", 200

        @self.app.route("/e_stop_action", methods=["POST"])
        def e_stop_action():
            return "", 200

        @self.app.route("/upload_file", methods=["POST"])
        def upload_file():
            """Saves the selected audio file to the system in ./audio/audio.wav"""
            if 'file' not in request.files:
                print("No file part in request")
                return "", 400
            file = request.files['file']
            if file.filename == "":
                print("No file selected")
                return "", 400

            # Save the file
            filepath = os.path.join(self.upload_folder, "audio.wav")
            file.save(filepath)
            print(f"File uploaded: {filepath}")
            return "", 200

        # Diameter submission handler
        @self.app.route("/submit_dia", methods=["POST"])
        def submit_dia():
            """Publishes the user entered outer and inner diameters to their corresponding topics"""
            data  = request.get_json()
            outer = data.get("outer", "")
            inner =  data.get("inner", "")

            # Publish the outer diameter to the outer_dia topic
            outer_dia = Int32()
            outer_dia.data = int(outer)
            self.publisher_outer_dia.publish(outer_dia)
            self.get_logger().info('Publishing Outer Dia: "%s"' % outer_dia.data)

            # Publish the inner diameter to the inner_dia topic
            inner_dia = Int32()
            inner_dia.data = int(inner)
            self.publisher_inner_dia.publish(inner_dia)
            self.get_logger().info('Publishing Inner Dia: "%s"' % inner_dia.data)

            return "", 200

        # Return to Home button action
        @self.app.route("/RTH_action", methods=["POST"])
        def RTH_action():
            return "", 200

        # Set Home button action
        @self.app.route("/set_home_action", methods=["POST"])
        def set_home_action():
            return "", 200

def main(args=None):
    try:
        with rclpy.init(args=args):
            web_interface = WebInterface()

            # Start Flask server in another thread
            flask_thread = threading.Thread(
                target=web_interface.app.run,
                kwargs={"host": web_interface.host, "port": web_interface.port, "debug": False},
                daemon=True
            )
            flask_thread.start()

            rclpy.spin(web_interface)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass