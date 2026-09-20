(function () {
  const t = window.rebotI18n ? window.rebotI18n.t : (k) => k;
  class ReBotRosClient extends EventTarget {
    constructor(options) {
      super();
      this.url = options && options.url ? options.url : '';
      this.namespace = options && options.namespace ? options.namespace : 'rebotarm';
      this.socket = null;
      this.connected = false;
      this.autoReconnect = true;
      this.reconnectDelay = 1400;
      this._subscriptions = new Map();
      this._advertisedTopics = new Set();
      this._pendingServices = new Map();
      this._pendingActions = new Map();
      this._nextId = 1;
      this._manualClose = false;
      this._lastMessageAt = new Map();
      this._connectSeq = 0;
    }

    connect(url) {
      if (url) this.url = url;
      this._manualClose = false;
      if (this.socket && this.socket.readyState === WebSocket.OPEN) {
        this._emitStatus('open', t('client.connected'));
        return;
      }
      if (this.socket && this.socket.readyState === WebSocket.CONNECTING) return;

      const seq = ++this._connectSeq;
      this._emitStatus('connecting', t('client.connecting', { url: this.url }));
      this.socket = new WebSocket(this.url);
      const socket = this.socket;

      socket.addEventListener('open', () => {
        if (seq !== this._connectSeq || socket !== this.socket) return;
        this.connected = true;
        this._emitStatus('open', t('client.connected'));
        this._resubscribe();
      });

      socket.addEventListener('message', (event) => {
        if (seq === this._connectSeq && socket === this.socket) this._handleMessage(event);
      });
      socket.addEventListener('error', () => {
        if (seq === this._connectSeq && socket === this.socket) this._emitStatus('error', t('client.wsError'));
      });
      socket.addEventListener('close', () => {
        if (seq !== this._connectSeq || socket !== this.socket) return;
        this.connected = false;
        this._rejectPendingServices(t('client.disconnected'));
        this._rejectPendingActions(t('client.disconnected'));
        this._emitStatus('closed', t('client.closed'));
        if (!this._manualClose && this.autoReconnect) {
          window.setTimeout(() => this.connect(), this.reconnectDelay);
        }
      });
    }

    disconnect() {
      this._manualClose = true;
      this.autoReconnect = false;
      this.connected = false;
      this._rejectPendingServices(t('client.disconnected'));
      this._rejectPendingActions(t('client.disconnected'));
      this._emitStatus('closed', t('client.closed'));
      if (this.socket) this.socket.close();
      this.socket = null;
    }

    subscribe(topic, type, callback, options) {
      const throttleRate = options && options.throttleRate ? options.throttleRate : 80;
      this._subscriptions.set(topic, { topic, type, callback, throttleRate });
      if (this.connected) this._sendSubscribe(topic, type, throttleRate);
    }

    unsubscribe(topic) {
      this._subscriptions.delete(topic);
      this._send({ op: 'unsubscribe', topic });
    }

    callService(service, type, args) {
      const id = this._id('service');
      return new Promise((resolve, reject) => {
        if (!this.connected) {
          reject(new Error(t('client.notConnected')));
          return;
        }
        this._pendingServices.set(id, { resolve, reject });
        this._send({
          op: 'call_service',
          id,
          service,
          type,
          args: args || {}
        });
      });
    }

    enable() {
      return this.callService(`/${this.namespace}/enable`, 'std_srvs/srv/Trigger', {});
    }

    disable() {
      return this.callService(`/${this.namespace}/disable`, 'std_srvs/srv/Trigger', {});
    }

    safeHome() {
      return this.callService(`/${this.namespace}/safe_home`, 'std_srvs/srv/Trigger', {});
    }

    startGravityCompensation() {
      return this.callService(`/${this.namespace}/gravity_compensation/start`, 'std_srvs/srv/Trigger', {});
    }

    stopGravityCompensation() {
      return this.callService(`/${this.namespace}/gravity_compensation/stop`, 'std_srvs/srv/Trigger', {});
    }

    gravityCompensationStatus() {
      return this.callService(`/${this.namespace}/gravity_compensation/status`, 'std_srvs/srv/Trigger', {});
    }

    setGripper(position, maxEffort) {
      return this.callService(`/${this.namespace}/gripper/set`, 'rebotarm_msgs/srv/SetGripper', {
        position,
        max_effort: maxEffort || 0
      });
    }

    releaseGripper() {
      return this.callService(`/${this.namespace}/gripper/release`, 'std_srvs/srv/Trigger', {});
    }

    holdGripper() {
      return this.callService(`/${this.namespace}/gripper/hold`, 'std_srvs/srv/Trigger', {});
    }

    startGripperAssist() {
      return this.callService(`/${this.namespace}/gripper/assist/start`, 'std_srvs/srv/Trigger', {});
    }

    gripperAssistStatus() {
      return this.callService(`/${this.namespace}/gripper/assist/status`, 'std_srvs/srv/Trigger', {});
    }

    moveToPose(pose, duration) {
      return this.sendActionGoal(`/${this.namespace}/move_to_pose`, 'rebotarm_msgs/action/MoveToPose', {
        target_pose: pose,
        duration: Number(duration) || 2
      });
    }

    solveMoveToPoseIK(pose) {
      return this.callService(`/${this.namespace}/move_to_pose_ik`, 'rebotarm_msgs/srv/MoveToPoseIK', {
        target_pose: pose
      });
    }

    followJointTrajectory(jointNames, points, options) {
      const expectedDuration = this._estimateTrajectoryDuration(points);
      const frameId = options && options.profile === 'teaching-replay'
        ? 'rebotarm_dm_teaching_replay'
        : '';
      return this.sendActionGoal(`/${this.namespace}/follow_joint_trajectory`, 'control_msgs/action/FollowJointTrajectory', {
        trajectory: {
          header: { stamp: { sec: 0, nanosec: 0 }, frame_id: frameId },
          joint_names: jointNames,
          points
        },
        goal_tolerance: [],
        path_tolerance: [],
        goal_time_tolerance: { sec: 0, nanosec: 0 }
      }, {
        timeoutMs: Math.max(30000, (expectedDuration + 10) * 1000)
      });
    }

    cancelActionGoals(actionName) {
      if (!this.connected) return false;
      let cancelled = false;
      this._pendingActions.forEach((pending, id) => {
        if (actionName && pending.action !== actionName) return;
        this._send({
          op: 'cancel_action_goal',
          id,
          action: pending.action
        });
        cancelled = true;
      });
      return cancelled;
    }

    sendActionGoal(actionName, actionType, goal, options) {
      const timeoutMs = Number(options && options.timeoutMs) || 30000;
      const id = this._id('action');
      return new Promise((resolve, reject) => {
        if (!this.connected) {
          reject(new Error(t('client.notConnected')));
          return;
        }
        const timer = window.setTimeout(() => {
          if (this._pendingActions.has(id)) {
            this._pendingActions.delete(id);
            reject(new Error(t('client.actionTimeout', { sec: timeoutMs / 1000, action: actionName })));
          }
        }, timeoutMs);
        this._pendingActions.set(id, {
          resolve: (value) => {
            window.clearTimeout(timer);
            resolve(value);
          },
          reject: (error) => {
            window.clearTimeout(timer);
            reject(error);
          },
          action: actionName
        });
        this._send({
          op: 'send_action_goal',
          id,
          action: actionName,
          action_type: actionType,
          args: goal || {},
          feedback: true
        });
      });
    }

    getRosTopics() {
      return this.callService('/rosapi/topics', 'rosapi_msgs/srv/Topics', {});
    }

    getRosServices() {
      return this.callService('/rosapi/services', 'rosapi_msgs/srv/Services', {});
    }

    getRosActionServers() {
      return this.callService('/rosapi/action_servers', 'rosapi_msgs/srv/GetActionServers', {});
    }

    _estimateTrajectoryDuration(points) {
      return (Array.isArray(points) ? points : []).reduce((maximum, point) => {
        const time = point && point.time_from_start;
        const seconds = Number(time && time.sec ? time.sec : 0)
          + Number(time && time.nanosec ? time.nanosec : 0) * 1e-9;
        return Number.isFinite(seconds) ? Math.max(maximum, seconds) : maximum;
      }, 0);
    }

    getLastMessageAt(topic) {
      return this._lastMessageAt.get(topic) || 0;
    }

    publishJointCommand(jointName, position, options) {
      const topic = `/${this.namespace}/joints/${jointName}/cmd`;
      const type = 'rebotarm_msgs/msg/JointMotorCmd';
      this.advertise(topic, type);
      this.publish(topic, {
        mode: options && typeof options.mode === 'number' ? options.mode : 1,
        use_pos: true,
        use_vel: false,
        use_kp: false,
        use_kd: false,
        use_tau: false,
        use_vlim: Boolean(options && typeof options.vlim === 'number'),
        pos: position,
        vel: 0,
        kp: 0,
        kd: 0,
        tau: 0,
        vlim: options && typeof options.vlim === 'number' ? options.vlim : 0,
        stamp: { sec: 0, nanosec: 0 }
      });
    }

    publishGripperCommand(position) {
      const topic = `/${this.namespace}/gripper/cmd`;
      const type = 'rebotarm_msgs/msg/JointMotorCmd';
      this.advertise(topic, type);
      this.publish(topic, {
        mode: 1,
        use_pos: true,
        use_vel: false,
        use_kp: false,
        use_kd: false,
        use_tau: false,
        use_vlim: false,
        pos: position,
        vel: 0,
        kp: 0,
        kd: 0,
        tau: 0,
        vlim: 0,
        stamp: { sec: 0, nanosec: 0 }
      });
    }

    publishTargetPose(pose) {
      const topic = `/${this.namespace}/mujoco/target_pose`;
      const type = 'geometry_msgs/msg/PoseStamped';
      this.advertise(topic, type);
      this.publish(topic, {
        header: {
          stamp: { sec: 0, nanosec: 0 },
          frame_id: 'base_link'
        },
        pose: {
          position: {
            x: Number(pose && pose.position ? pose.position.x : pose && pose.x) || 0,
            y: Number(pose && pose.position ? pose.position.y : pose && pose.y) || 0,
            z: Number(pose && pose.position ? pose.position.z : pose && pose.z) || 0
          },
          orientation: pose && pose.orientation ? pose.orientation : { x: 0, y: 0, z: 0, w: 1 }
        }
      });
    }

    advertise(topic, type) {
      if (this._advertisedTopics.has(topic)) return;
      this._advertisedTopics.add(topic);
      this._send({ op: 'advertise', topic, type });
    }

    publish(topic, msg) {
      this._send({ op: 'publish', topic, msg });
    }

    _resubscribe() {
      this._subscriptions.forEach((sub) => {
        this._sendSubscribe(sub.topic, sub.type, sub.throttleRate);
      });
    }

    _sendSubscribe(topic, type, throttleRate) {
      this._send({
        op: 'subscribe',
        id: this._id('sub'),
        topic,
        type,
        throttle_rate: throttleRate,
        queue_length: 1
      });
    }

    _handleMessage(event) {
      let data;
      try {
        data = JSON.parse(event.data);
      } catch (error) {
        this._emitStatus('error', t('client.badMsg'));
        return;
      }

      if (data.op === 'publish') {
        this._lastMessageAt.set(data.topic, Date.now());
        const sub = this._subscriptions.get(data.topic);
        if (sub) sub.callback(data.msg, data.topic);
        return;
      }

      if (data.op === 'service_response') {
        const pending = this._pendingServices.get(data.id);
        if (!pending) return;
        this._pendingServices.delete(data.id);
        if (data.result === false) {
          pending.reject(new Error(data.values && data.values.message ? data.values.message : t('client.serviceFailed')));
        } else {
          pending.resolve(data.values || {});
        }
        return;
      }

      if (data.op === 'action_feedback') {
        const pending = this._pendingActions.get(data.id);
        if (!pending) return;
        this.dispatchEvent(new CustomEvent('action-feedback', {
          detail: {
            id: data.id,
            action: data.action || pending.action,
            values: data.values || {}
          }
        }));
        return;
      }

      if (data.op === 'action_result') {
        const pending = this._pendingActions.get(data.id);
        if (!pending) return;
        this._pendingActions.delete(data.id);
        if (data.result === false) {
         const message = typeof data.values === 'string'
           ? data.values
            : (data.values && data.values.message ? data.values.message : t('client.actionFailed'));
         pending.reject(new Error(message));
          return;
        }
        pending.resolve({
          ...(data.values || {}),
          status: data.status,
          action: data.action || pending.action,
          completed: true
        });
      }
    }

    _send(payload) {
      if (!this.socket || this.socket.readyState !== WebSocket.OPEN) return;
      this.socket.send(JSON.stringify(payload));
    }

    _rejectPendingServices(message) {
      this._pendingServices.forEach((pending) => pending.reject(new Error(message)));
      this._pendingServices.clear();
    }

    _rejectPendingActions(message) {
      this._pendingActions.forEach((pending) => pending.reject(new Error(message)));
      this._pendingActions.clear();
    }

    _id(prefix) {
      const id = `${prefix}:${this._nextId}`;
      this._nextId += 1;
      return id;
    }

    _uuid() {
      const values = new Uint8Array(16);
      if (window.crypto && typeof window.crypto.getRandomValues === 'function') {
        window.crypto.getRandomValues(values);
      } else {
        for (let i = 0; i < values.length; i += 1) {
          values[i] = Math.floor(Math.random() * 256);
        }
      }
      return Array.from(values);
    }

    _emitStatus(state, message) {
      this.dispatchEvent(new CustomEvent('status', { detail: { state, message } }));
    }
  }

  window.ReBotRosClient = ReBotRosClient;
})();
