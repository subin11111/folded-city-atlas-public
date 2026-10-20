#!/usr/bin/env python3
"""Show the actual installed rqt_graph plugin and save its live DDS graph.

The native widget screenshot and DOT come from rqt_graph, not a drawn mockup.
The separate JSON inventory records endpoints and message types from rclpy.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

import rclpy
from python_qt_binding.QtCore import QObject, QTimer
from python_qt_binding.QtWidgets import QApplication, QMainWindow
from rqt_graph.ros_graph import RosGraph
from rqt_graph.dotcode import NODE_TOPIC_ALL_GRAPH, NODE_NODE_GRAPH

from optimize_session import sha256_file, utc_now


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--hold',type=float,default=180.)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    app=QApplication([]);rclpy.init();node=rclpy.create_node('rqt_live_graph_capture_'+str(os.getpid()))
    window=QMainWindow();window.setWindowTitle('rqt_graph · ACTUAL recorded replay nodes · not online final mesh')
    window.resize(1560,950)

    class Context(QObject):
        def __init__(self):
            super().__init__();self.node=node
        def serial_number(self): return 1
        def add_widget(self,widget): window.setCentralWidget(widget)

    context=Context();plugin=RosGraph(context);widget=plugin._widget
    widget.filter_line_edit.setText('/.*')
    widget.topic_filter_line_edit.setText('/folded_city_demo/.*,/tf.*')
    widget.graph_type_combo_box.setCurrentIndex(2)
    for field in ('quiet_check_box','dead_sinks_check_box','leaf_topics_check_box','group_image_check_box','group_tf_check_box','hide_tf_nodes_check_box'):
        getattr(widget,field).setChecked(False)
    plugin.initialized=True
    plugin._refresh_rosgraph()
    window.show();start=time.monotonic();saved={'done':False}

    def capture():
        plugin._update_rosgraph()
        detailed=plugin.dotcode_generator.generate_dotcode(
            rosgraphinst=plugin._graph,ns_filter=widget.filter_line_edit.text(),
            topic_filter=widget.topic_filter_line_edit.text(),graph_mode=NODE_TOPIC_ALL_GRAPH,
            dotcode_factory=plugin.dotcode_factory,orientation='TB',quiet=False)
        plugin._update_graph_view(detailed);plugin._fit_in_view()
        nodes=node.get_node_names_and_namespaces()
        wanted=[(n,ns) for n,ns in nodes if ns=='/folded_city_demo']
        if len(wanted)<8:
            if time.monotonic()-start>45: raise RuntimeError('Eight real replay nodes not discovered')
            return
        if time.monotonic()-start<5: return
        dot=plugin._current_dotcode
        if not dot or 'capture_db_replay' not in dot: return
        (a.output/'rqt_graph.dot').write_text(dot)
        for extension in ('svg','png'):
            subprocess.run(['dot','-T'+extension,str(a.output/'rqt_graph.dot'),'-o',str(a.output/('rqt_graph.'+extension))],check=True)
        nodes_only=plugin.dotcode_generator.generate_dotcode(
            rosgraphinst=plugin._graph,ns_filter=widget.filter_line_edit.text(),
            topic_filter=widget.topic_filter_line_edit.text(),graph_mode=NODE_NODE_GRAPH,
            dotcode_factory=plugin.dotcode_factory,orientation='TB',quiet=False)
        (a.output/'rqt_nodes_only.dot').write_text(nodes_only)
        subprocess.run(['dot','-Tpng',str(a.output/'rqt_nodes_only.dot'),'-o',str(a.output/'rqt_nodes_only.png')],check=True)
        window.grab().save(str(a.output/'rqt_native_window.png'))
        inventory=[]
        for name,namespace in sorted(nodes):
            inventory.append({'name':name,'namespace':namespace,
                'publishers':node.get_publisher_names_and_types_by_node(name,namespace),
                'subscribers':node.get_subscriber_names_and_types_by_node(name,namespace)})
        result={'created_utc':utc_now(),'actual_rqt_graph_plugin':True,'inventory':inventory,
            'topic_filter':'/folded_city_demo/.*,/tf.*','all_replay_nodes_visible':True,
            'source':'live DDS discovery, installed rqt_graph plugin',
            'scope':'ROS processing graph, not SLAM keyframe graph; offline scripts are not pretended to be live ROS nodes',
            'native_screenshot_sha256':sha256_file(a.output/'rqt_native_window.png')}
        (a.output/'ros_graph_inventory.json').write_text(json.dumps(result,indent=2)+'\n')
        if not saved['done']: print('ACTUAL_RQT_GRAPH_SAVED',flush=True)
        saved['done']=True
        if time.monotonic()-start>a.hold: app.quit()

    timer=QTimer();timer.timeout.connect(capture);timer.start(3000)
    app.exec_();plugin.shutdown_plugin();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
