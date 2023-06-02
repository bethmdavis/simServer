# syntax=docker/dockerfile:1

FROM python:3.8-slim-buster

WORKDIR /python-docker

COPY requirements.txt requirements.txt
RUN pip3 install -r requirements.txt

COPY . .

ENV FLASK_APP=app.py

#CMD [ "python3", "-m" , "flask", "run", "--host=0.0.0.0"]

CMD ["gunicorn","--workers","5","--timeout","0", "--limit-request-line", "0", "--bind", "0.0.0.0:5000", "wsgi:app"]