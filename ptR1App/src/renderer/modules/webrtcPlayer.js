//webrtc-player.js
export class WebRTCPlayer {
    constructor(url, videoElement, statusElement, overlayElement) {
        this.url = url;
        this.videoElement = videoElement;
        this.overlayElement = overlayElement;
        this.statusElement = statusElement;
        this.peerConnection = null;
    }

    /**
     * Starts the WebRTC connection process.
     */
    async connect() {
        this._updateStatus('Initializing WebRTC connection...');

        if (this.overlayElement) {
            this.overlayElement.classList.add('hidden');
        }
        
        if (this.peerConnection) {
            this.disconnect();
        }

        //Create a new RTCPeerConnection
        this.peerConnection = new RTCPeerConnection();

        //Handle incoming tracks
        this.peerConnection.ontrack = (event) => {
            this._updateStatus('Stream connected!');
            console.log(`Track received: ${event.track.kind}`);
            if (!this.videoElement.srcObject) {
                this.videoElement.srcObject = new MediaStream();
            }
            this.videoElement.srcObject.addTrack(event.track);
            this.videoElement.play().catch(e => console.warn("Auto-play blocked:", e));
        };
        
        //Define the media we want to receive
        this.peerConnection.addTransceiver('video', { direction: 'recvonly' });
        // this.peerConnection.addTransceiver('audio', { direction: 'recvonly' }); // Uncomment for audio

        try {
            //Create and set local SDP offer
            const offer = await this.peerConnection.createOffer();
            await this.peerConnection.setLocalDescription(offer);
            this._updateStatus('Sending offer to server...');

            //Send offer to MediaMTX WHEP endpoint
            const response = await fetch(this.url, {
                method: 'POST',
                headers: { 'Content-Type': 'application/sdp' },
                body: this.peerConnection.localDescription.sdp
            });

            if (!response.ok) {
                const errorText = await response.text();
                throw new Error(`Server responded with status ${response.status}: ${errorText}`);
            }

            //Receive and set remote SDP answer
            this._updateStatus('Received answer, establishing connection...');
            const answerSdp = await response.text();
            await this.peerConnection.setRemoteDescription(
                new RTCSessionDescription({ type: 'answer', sdp: answerSdp })
            );

            console.log('WebRTC connection established successfully!');

        } catch (error) {
            console.error('Failed to start WebRTC stream:', error);
            this._updateStatus(`Error: ${error.message}`);
            this.disconnect(); // Clean up on failure
        }
    }

    /**
     * Disconnects the WebRTC connection.
     */
    disconnect() {
        // ปิด PeerConnection
        if (this.peerConnection) {
            this.peerConnection.close();
            this.peerConnection = null;
        }

        // เคลียร์ Video Element
        if (this.videoElement.srcObject) {
            this.videoElement.srcObject.getTracks().forEach(track => track.stop()); // หยุด Track ทั้งหมด
            this.videoElement.srcObject = null; // ล้างค่าทิ้ง
        }
        if (this.overlayElement) {
            this.overlayElement.classList.remove('hidden');
        }
        if (this._frameCallbackId) {
        this.videoElement.cancelVideoFrameCallback(this._frameCallbackId);
        this._frameCallbackId = null;
}

        this._updateStatus('Disconnected.');
        console.log('WebRTC connection closed and cleaned up.');

        // ล้าง interval การตรวจสอบ delay
        if (this._delayInterval) clearInterval(this._delayInterval);
        this._tmpCanvas = null; 
        this._tmpCtx = null;
        if (window.Tesseract) {
            Tesseract.terminate();
        }
    }
    _updateStatus(message) {
        if (this.statusElement) {
            this.statusElement.innerText = message;
        }
    }
    
}

