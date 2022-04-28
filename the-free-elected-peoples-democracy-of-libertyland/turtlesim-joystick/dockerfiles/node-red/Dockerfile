FROM nodered/node-red:latest

USER root
RUN apk add --update npm
# USER node-red

# Copy package.json to the WORKDIR so npm builds all
# of your added nodes modules for Node-RED
# ADD node-red-data/data/ /data
# RUN chown -R root:root /data
# RUN chmod 755 /data
# WORKDIR /data
# RUN npm install --unsafe-perm --no-update-notifier --no-fund --only=production

# Copy _your_ Node-RED project files into place
# NOTE: This will only work if you DO NOT later mount /data as an external volume.
#       If you need to use an external volume for persistence then
#       copy your settings and flows files to that volume instead.
# COPY settings.js /data/settings.js
# COPY flows_cred.json /data/flows_cred.json
# COPY flows.json /data/flows.json

# You should add extra nodes via your package.json file but you can also add them here:
#WORKDIR /usr/src/node-red
#RUN npm install node-red-node-smoot
# ENTRYPOINT ["npm" "--no-update-notifier" "--no-fund" "start" "--cache" "/data/.npm" "--" "--userDir" "/data"]
ENTRYPOINT ["npm" "install" "&&" "node" "$NODE_OPTIONS" "node_modules/node-red/red.js" "$FLOWS" "--userDir" "/data"]