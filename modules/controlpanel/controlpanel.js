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
                    width: 50%;
                    margin: auto;
                    padding: 20px;
                    border: 5px solid #000000;
                    border-radius: 10px;
                    box-shadow: 2px 2px 10px rgba(0,0,0,0.1);
                    background-color: rgba(255, 255, 255, 0.85);
                    text-align: center;
                }

                .control-group {
                    margin: 15px 0;
                    display: flex;
                    align-items: center;
                    justify-content: center;
                }

                .control-group label {
                    flex: 1;
                    text-align: right;
                    margin-right: 10px;
                }

                .control-group input[type="range"] {
                    flex: 2;
                    margin-left: 10px;
                }

                .button-group {
                    display: flex;
                    justify-content: center;
                    gap: 10px;
                }

                button {
                    padding: 10px 20px;
                    margin: 5px;
                    border: none;
                    background-color: #8C1D40;
                    color: white;
                    border-radius: 5px;
                    cursor: pointer;
                }

                button:hover {
                    background-color: #b35875;
                }
            </style>
            <div class="container">
                <h1>Simulation Controller</h1>
                <div class="control-group">
                    <label for="predator-speed">Predator Speed:</label>
                    <input type="range" id="predator-speed" min="1" max="10" step="1">
                </div>
                <div class="control-group">
                    <label for="prey-speed">Prey Speed:</label>
                    <input type="range" id="prey-speed" min="1" max="10" step="1">
                </div>
                <div class="button-group">
                    <button id="start-btn">Start Simulation</button>
                    <button id="stop-btn">Stop Simulation</button>
                    <button id="reset-btn">Reset</button>
                </div>
                <div class="control-group">
                    <h3>Status: <span id="status">Idle</span></h3>
                </div>
            </div>
        `;
    }

    connectedCallback() {
        this.shadowRoot.getElementById('start-btn').addEventListener('click', this.startSimulation.bind(this));
        this.shadowRoot.getElementById('stop-btn').addEventListener('click', this.stopSimulation.bind(this));
        this.shadowRoot.getElementById('reset-btn').addEventListener('click', this.resetSimulation.bind(this));
    }

    disconnectedCallback() {
        this.shadowRoot.getElementById('start-btn').removeEventListener('click', this.startSimulation.bind(this));
        this.shadowRoot.getElementById('stop-btn').removeEventListener('click', this.stopSimulation.bind(this));
        this.shadowRoot.getElementById('reset-btn').removeEventListener('click', this.resetSimulation.bind(this));
    }

    startSimulation() {
        this.shadowRoot.getElementById('status').textContent = 'Running';
    }

    stopSimulation() {
        this.shadowRoot.getElementById('status').textContent = 'Stopped';
    }

    resetSimulation() {
        this.shadowRoot.getElementById('status').textContent = 'Idle';
    }
}

customElements.define('control-panel', ControlPanel);