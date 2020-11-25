#!/usr/bin/env python
import rospy
import random
import threading
import webbrowser


from flask import *
import json
import copy

import requests

app = Flask(__name__)
app.secret_key="sdf"
i=0




@app.route('/test2/<name>')
def test2(name):
    
    return render_template('test.html',t="5")

@app.route('/', methods=['GET','POST'])
@app.route('/test', methods=['GET','POST'])
def test():
    print('during view')
    global i
    i+=1

   
    s=request.form

    t=s.get("5")
    
    return redirect(url_for('test2',name=t))
    #return render_template('test.html',t=t)
    

def test2():
    pass
   
'''
@app.teardown_request
def show_teardown(exception):
    print('after with block')

with app.test_request_context():
    print('during with block')

# teardown functions are called after the context with block exits

with app.test_client() as client:
    client.get('/test')
    # the contexts are not popped even though the request ended
    print(request.path)

# the contexts are popped and teardown functions are called after
# the client with block exists
'''


if __name__ == '__main__':

    port = 5000# + random.randint(0, 999)
    url = "http://127.0.0.1:{0}".format(port)
    threading.Timer(1.25, lambda: webbrowser.open(url) ).start()

    #app.run(port=port, debug=False)
    app.run()

    
    
