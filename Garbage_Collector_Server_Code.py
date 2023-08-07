from flask import Flask, jsonify, request
from jetbot import Robot

robot = Robot()

app = Flask(__name__)

# Başlangıç verileri
data = {
    'CopTuru': 'Henuz bir islem yapilmiyor :( ',
    'CopSayisi': 60
}

@app.route('/veriler', methods=['GET'])
def get_data():
    return jsonify(data)

@app.route('/veriler', methods=['POST'])
def update_data():
    global data  # data değişkenini global olarak işaretleyin

    new_data = request.json  # Kullanıcıdan gelen verileri al

    # Verileri güncelle
    if 'CopTuru' in new_data:
        data['CopTuru'] = new_data['CopTuru']
    if 'CopSayisi' in new_data:
        data['CopSayisi'] = new_data['CopSayisi']

    return jsonify({'message': 'Veriler güncellendi.'})
@app.route('/ileri', methods=['POST'])
def move_forward():
    
    robot.forward(0.2)

@app.route('/dur', methods=['POST'])
def stop():
    
    robot.stop()
    
@app.route('/geri', methods=['POST'])
def move_backward():
    
    robot.backward(0.2)

def main():
    app.run(host='192.168.10.34', port=5555)

if __name__ == '__main__':
    main()