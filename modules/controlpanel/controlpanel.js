class ControlPanel extends HTMLElement {
    constructor() {
        super();
        this.attachShadow({ mode: 'open' });
        this.shadowRoot.innerHTML = `
            <style>
                body {
                    font-family: Cambria, Arial, sans-serif;
                    text-align: center;
                    margin: 20px;
                    background-size: cover;
                    background-position: center;
                    background-attachment: fixed;
                }

                .container {
                    width: 80%;
                    margin: auto;
                    padding: 20px;
                    border: 5px solid #000000;
                    border-radius: 10px;
                    box-shadow: 2px 2px 10px rgba(0,0,0,0.1);
                    background-color: rgba(255, 255, 255, 0.85);
                    text-align: center;
                }

                .screen {
                    position: relative;
                    width: 100%;
                    height: 300px;
                    border: 2px solid #000000;
                    border-radius: 5px;
                    background-color: #e0e0e0;
                    margin-bottom: 20px;
                }

                .overlay {
                    position: absolute;
                    top: 10px;
                    left: 10px;
                    width: 100px;
                    height: 100px;
                    border: 2px solid #000000;
                    background-color: #ffffff;
                    font-size: 12px;
                    text-align: center;
                    line-height: 100px;
                }

                .slam-view {
                    position: absolute;
                    top: 10px;
                    right: 10px;
                    width: 150px;
                    height: 150px;
                    border: 2px solid #000000;
                    background-color: #ffffff;
                    font-size: 12px;
                    text-align: center;
                    line-height: 150px;
                }

                .button-grid {
                    display: flex;
                    justify-content: space-around;
                    margin-top: 20px;
                }

                .button-grid button {
                    padding: 15px;
                    border: none;
                    background-color: #8C1D40;
                    color: white;
                    border-radius: 5px;
                    cursor: pointer;
                    font-size: 16px;
                    width: 150px;
                }

                .button-grid button:hover {
                    background-color: #b35875;
                }

                .start-button {
                    margin-top: 20px;
                    padding: 20px;
                    font-size: 18px;
                    background-color: #FFC627;
                    border: none;
                    color: white;
                    border-radius: 5px;
                    cursor: pointer;
                    width: 300px;
                    color: black;
                }

                .start-button:hover {
                    background-color:rgb(215, 167, 34);
                    color: black;
                }
            </style>
            <div class="container">
                <h1>Simulation Controller</h1>
                <div class="screen" id="camera-screen">
                    <div class="overlay">IMU DATA</div>
                    <div class="slam-view">SLAM VIEW</div>
                </div>
                <div class="button-grid">
                    <button id="imu-btn">RPi IMU</button>
                    <button id="camera-btn">RPi Camera</button>
                    <button id="esp-btn">ESP32 IMU</button>
                </div>
                <button class="start-button" id="start-predator-prey-btn">START</button>
            </div>
        `;
    }

    connectedCallback() {
        this.shadowRoot.getElementById('imu-btn').addEventListener('click', this.handleIMUClick.bind(this));
        this.shadowRoot.getElementById('camera-btn').addEventListener('click', this.handleCameraClick.bind(this));
        this.shadowRoot.getElementById('esp-btn').addEventListener('click', this.handleESPClick.bind(this));
        this.shadowRoot.getElementById('start-predator-prey-btn').addEventListener('click', this.startPredatorPrey.bind(this));
    }

    disconnectedCallback() {
        this.shadowRoot.getElementById('imu-btn').removeEventListener('click', this.handleIMUClick.bind(this));
        this.shadowRoot.getElementById('camera-btn').removeEventListener('click', this.handleCameraClick.bind(this));
        this.shadowRoot.getElementById('esp-btn').removeEventListener('click', this.handleESPClick.bind(this));
        this.shadowRoot.getElementById('start-predator-prey-btn').removeEventListener('click', this.startPredatorPrey.bind(this));
    }

    handleIMUClick() {
        console.log('IMU button clicked');
    }

    handleCameraClick() {
        console.log('Camera button clicked');
    }

    handleESPClick() {
        console.log('ESP button clicked');
    }

    startPredatorPrey() {
        console.log('Starting Predator vs Prey simulation');
    }
}

customElements.define('control-panel', ControlPanel);