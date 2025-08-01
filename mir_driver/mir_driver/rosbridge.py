## @package rosbridge
#  This module provides a WebSocket-based bridge for communication with ROS.

import websocket
import threading

import json
import traceback
import time

import string
import random

## @class RosbridgeSetup
#  @brief A class to set up and manage the connection to the ROS bridge.
class RosbridgeSetup():
    ## Constructor
    #  @param host The host address of the ROS bridge.
    #  @param port The port number of the ROS bridge.
    def __init__(self, host, port):
        self.callbacks = {}
        self.service_callbacks = {}
        self.resp = None
        self.connection = RosbridgeWSConnection(host, port)
        self.connection.registerCallback(self.onMessageReceived)

    ## Publish a message to a specified topic.
    #  @param topic The topic to publish to.
    #  @param obj The message object to publish.
    def publish(self, topic, obj):
        pub = {"op": "publish", "topic": topic, "msg": obj}
        self.send(pub)

    ## Subscribe to a specified topic.
    #  @param topic The topic to subscribe to.
    #  @param callback The callback function to handle incoming messages.
    #  @param throttle_rate The rate at which messages are throttled (optional).
    def subscribe(self, topic, callback, throttle_rate=-1):
        if self.addCallback(topic, callback):
            sub = {"op": "subscribe", "topic": topic}
            if throttle_rate > 0:
                sub['throttle_rate'] = throttle_rate

            self.send(sub)

    ## Unhook a callback from all topics.
    #  @param callback The callback function to unhook.
    def unhook(self, callback):
        keys_for_deletion = []
        for key, values in self.callbacks.items():
            for value in values:
                if callback == value:
                    print("Found!")
                    values.remove(value)
                    if len(values) == 0:
                        keys_for_deletion.append(key)

        for key in keys_for_deletion:
            self.unsubscribe(key)
            self.callbacks.pop(key)

    ## Unsubscribe from a specified topic.
    #  @param topic The topic to unsubscribe from.
    def unsubscribe(self, topic):
        unsub = {"op": "unsubscribe", "topic": topic}
        self.send(unsub)

    ## Call a ROS service.
    #  @param serviceName The name of the service to call.
    #  @param callback The callback function to handle the service response (optional).
    #  @param msg The message to send with the service call (optional).
    #  @return The response from the service call if no callback is provided.
    def callService(self, serviceName, callback=None, msg=None):
        id = self.generate_id()
        call = {"op": "call_service", "id": id, "service": serviceName}
        if msg is not None:
            call['args'] = msg

        if callback is None:
            self.resp = None

            def internalCB(msg):
                self.resp = msg
                return None

            self.addServiceCallback(id, internalCB)
            self.send(call)

            while self.resp is None:
                time.sleep(0.01)

            return self.resp

        self.addServiceCallback(id, callback)
        self.send(call)
        return None

    ## Send a message to the ROS bridge.
    #  @param obj The message object to send.
    def send(self, obj):
        try:
            self.connection.sendString(json.dumps(obj))
        except Exception:
            traceback.print_exc()
            raise

    ## Generate a unique identifier.
    #  @param chars The number of characters in the identifier (default is 16).
    #  @return A unique identifier string.
    def generate_id(self, chars=16):
        return ''.join(random.SystemRandom().choice(
            string.ascii_letters + string.digits) for _ in range(chars))

    ## Add a callback for a service response.
    #  @param id The unique identifier for the service call.
    #  @param callback The callback function to handle the service response.
    def addServiceCallback(self, id, callback):
        self.service_callbacks[id] = callback

    ## Add a callback for a topic.
    #  @param topic The topic to add the callback for.
    #  @param callback The callback function to handle incoming messages.
    #  @return True if the callback was added, False if it already exists.
    def addCallback(self, topic, callback):
        if topic in self.callbacks:
            self.callbacks[topic].append(callback)
            return False

        self.callbacks[topic] = [callback]
        return True

    ## Check if the connection to the ROS bridge is established.
    #  @return True if connected, False otherwise.
    def is_connected(self):
        return self.connection.connected

    ## Check if there was an error in the connection.
    #  @return True if there was an error, False otherwise.
    def is_errored(self):
        return self.connection.errored

    ## Gracefully shut down the connection to the ROS bridge.
    def closing(self):
        self.connection.close_()

    ## Handle incoming messages from the ROS bridge.
    #  @param message The message received from the ROS bridge.
    def onMessageReceived(self, message):
        try:
            # Load the string into a JSON object
            obj = json.loads(message)
            # print ("Received: ", obj)

            if 'op' in obj:
                option = obj['op']
                if option == "publish":  # A message from a topic we have subscribed to..
                    topic = obj["topic"]
                    msg = obj["msg"]
                    if topic in self.callbacks:
                        for callback in self.callbacks[topic]:
                            try:
                                callback(msg)
                            except Exception:
                                print("exception on callback",
                                      callback, "from", topic)
                                traceback.print_exc()
                                raise
                elif option == "service_response":
                    if "id" in obj:
                        id = obj["id"]
                        values = obj["values"]
                        if id in self.service_callbacks:
                            try:
                                # print 'id:', id, 'func:', self.service_callbacks[id]
                                self.service_callbacks[id](values)
                            except Exception:
                                print("exception on callback ID:", id)
                                traceback.print_exc()
                                raise
                    else:
                        print("Missing ID!")
                else:
                    print("Recieved unknown option - it was: ", option)
            else:
                print("No OP key!")
        except Exception:
            print("exception in onMessageReceived")
            print("message", message)
            traceback.print_exc()
            raise

## @class RosbridgeWSConnection
#  @brief A class to manage the WebSocket connection to the ROS bridge.
class RosbridgeWSConnection():
    ## Constructor
    #  @param host The host address of the ROS bridge.
    #  @param port The port number of the ROS bridge.
    def __init__(self, host, port):
        self.ws = websocket.WebSocketApp(("ws://%s:%d/" % (host, port)),
                                         on_message=self.on_message,
                                         on_error=self.on_error,
                                         on_close=self.on_close)
        self.ws.on_open = self.on_open
        self.run_thread = threading.Thread(target=self.run)
        self.run_thread.start()
        self.connected = False
        self.errored = False
        self.callbacks = []

    ## Handle the WebSocket connection opening.
    #  @param ws The WebSocket object.
    def on_open(self, ws):
        print("### ROS bridge already connected ###")
        self.connected = True

    ## Send a string message over the WebSocket connection.
    #  @param message The message string to send.
    def sendString(self, message):
        if not self.connected:
            print("Error: not connected, could not send message")
            # TODO: throw exception
        else:
            self.ws.send(message)

    ## Handle errors in the WebSocket connection.
    #  @param ws The WebSocket object.
    #  @param error The error message.
    def on_error(self, ws, error):
        self.errored = True
        print("Error: %s" % error)

    ## Handle the WebSocket connection closing.
    #  @param ws The WebSocket object.
    '''
    def on_close(self, ws):
        self.connected = False
        print("ROS bridge closed")
    ''' 
    def on_close(self, ws, close_status_code, close_msg):
        self.connected = False
        print("ROS bridge closed")

    ## Close the WebSocket connection gracefully.
    def close_(self):
        self.ws.close()
        print("### ROS bridge closed ###")

    ## Run the WebSocket connection in a separate thread.
    #  @param args Additional arguments.
    def run(self, *args):
        self.ws.run_forever()

    ## Handle incoming messages from the WebSocket connection.
    #  @param ws The WebSocket object.
    #  @param message The message received.
    def on_message(self, ws, message):
        # Call the handlers
        for callback in self.callbacks:
            callback(message)

    ## Register a callback for handling incoming messages.
    #  @param callback The callback function to register.
    def registerCallback(self, callback):
        self.callbacks.append(callback)
