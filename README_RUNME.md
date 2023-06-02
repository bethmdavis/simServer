Running the Server
---
Once Python and all its dependencies are installed, the following commands should be entered at the command line in order to run the server:
- `conda create -n robotarium python=3.10`
- `conda activate robotarium`
- `pip install -r requirements.txt`
- `python app.py`

This will start the Flask application server and send its output to the terminal. As you interact with the Simulator website, you will see the results of that interaction in realtime.

To see the animations, go to the `static/uploads` folder; the `mp4` files are in there. The images that are put together to form the animations are in `static/images`.
