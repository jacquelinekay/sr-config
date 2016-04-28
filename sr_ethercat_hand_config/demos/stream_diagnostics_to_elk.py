from datetime import datetime
from elasticsearch import Elasticsearch
import logging
import rospy
from diagnostic_msgs.msg import DiagnosticArray
# from rospy_message_converter import message_converter

tracer = logging.getLogger('elasticsearch.trace')
tracer.setLevel(logging.INFO)
tracer.addHandler(logging.FileHandler('/tmp/es_trace.log'))

es = Elasticsearch([{'host': 'localhost', 'port': 9200}])
es.indices.create(index='shadow-diagnostics', ignore=400)

rospy.init_node(name="diag_to_elg")


def diag_cb(msg):
    # Tpdo convert header stamp to proper time stamp format
    for status in msg.status:
        body = {}
        body["@timestamp"] = datetime.now()

        if status.level == status.OK:
            body["diagnostic_levels"] = 'OK'
        elif status.level == status.WARN:
            body["diagnostic_levels"] = 'WARN'
        elif status.level == status.ERROR:
            body["diagnostic_levels"] = 'ERROR'
        elif status.level == status.STALE:
            body["diagnostic_levels"] = 'STALE'

        body["diagnostic_name"] = status.name
        body["diagnostic_message"] = status.message
        body["diagnostic_hardware_id"] = status.hardware_id

        for value in status.values:
            # Make sure serial is kept as a string
            if value.key == "Serial Number":
                body[value.key] = value.value
            else:
                try:
                    body[value.key] = float(value.value)
                except ValueError:
                    body[value.key] = value.value

        es.index(index="shadow-diagnostics", doc_type="diagnostics",
                 body=body)

sub = rospy.Subscriber("/diagnostics", DiagnosticArray, diag_cb,
                       queue_size=1)
rospy.spin()
