import matplotlib
matplotlib.use('Agg') # disable errors from flask

from flask import Flask, url_for, request, send_from_directory
import os
import time
import threading
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

app.config['UPLOAD_FOLDER'] = "static/uploads"
app.config['MAX_CONTENT_LENGTH'] = 16 * 1024 * 1024

# my background thread
class MyWorker():

    def __init__(self, message):
        self.message = message

        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True
        thread.start()

    def run(self):

        # do something
        text_file = open("static/text/Code.txt", "r")

        #read whole file to a string
        code = text_file.read()
        print(code)
        exec(code,{})


@app.route('/')
def hello():
    return '<h1>Hello Robotarium!</h1>'

# this works but no video
#@app.route('/static/<path:path>')
#def send_report(path):
#    return send_from_directory('static', path)

# this works and video is downloaded as an attachment
#@app.route('/static/uploads/<path:filename>')
#def download_file(filename):
#    return send_from_directory(app.config['UPLOAD_FOLDER'],
#                               filename, as_attachment=True)

# this works but no video
@app.route('/static/uploads/<path:filename>')
def download_file(filename):
    return send_from_directory(app.config['UPLOAD_FOLDER'],
                               filename)

@app.route('/favicon.ico') 
def favicon(): 
    return send_from_directory(os.path.join(app.root_path, 'static'), 'favicon.ico', mimetype='image/vnd.microsoft.icon')


@app.route('/simulationVideo',methods = ['POST'])
def login():
    if request.method == 'POST':

        code = request.form['code']

        exec(code,{})
        
        with open('static/text/timestamp.txt') as f:
            timestamp = f.readlines()[0]
        
        return timestamp

@app.route('/track',methods = ['GET'])
def track():
    if request.method == 'GET':
        time.sleep(1)
        if os.path.isfile('images/1.png'):
            return '200'
        else:
            return '100'
      

@app.route('/display_video/video.mp4')
def display_video():

	#print('display_video filename: ' + filename)
	return url_for('static', filename='uploads/' + 'video.mp4', code=301)

if __name__ == "__main__":
    app.run(debug=True)
