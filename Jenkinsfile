pipeline {
    agent {
        kubernetes {
            yaml """
            apiVersion: v1
            kind: Pod
            metadata:
              labels:
                job: polaris_gem_simulation
            spec:
              containers:
              - name: docker
                image: docker:25.0.5-git
                securityContext:
                  privileged: true
                command: ["dockerd-entrypoint.sh"]
                args: ["--host=tcp://0.0.0.0:2375", "--host=unix:///var/run/docker.sock"]
            """
        }
    }

    parameters {
        string(name: 'DOCKER_IMAGE', defaultValue: 'pritishbisht/polaris_gem_sim', description: 'Docker image name')
        string(name: 'EMAIL_RECIPIENTS', defaultValue: 'pritishbisht123@gmail.com', description: 'Email recipient of job status')
        string(name: 'DURATION', defaultValue: '30', description: 'Test duration')
        string(name: 'ERROR_THRESHOLD', defaultValue: '1.0', description: 'Acceptable cross-track error threshold')
    }

    environment {
        IMAGE_NAME = "${params.DOCKER_IMAGE}"
        TAG_LATEST = "${IMAGE_NAME}:latest"
        TAG_BUILD = "${IMAGE_NAME}:build-${BUILD_NUMBER}"
        CONTAINER_NAME = "test_sim_container"
        TEST_DURATION = "${params.DURATION}"
        ERROR_THRESHOLD = "${params.ERROR_THRESHOLD}"
        imageBuilt = false
    }

    stages {
        
        stage('Checkout') {
            steps {
                container('docker') {
                    git branch: 'main', url: 'https://github.com/pritishbisht/polaris_cte_test.git'
                }
            }
        }

        stage('Build or Pull Docker Image') {
            steps {
                container('docker') {
                    script {
                        // Resolve ownership  
                        sh 'git config --global --add safe.directory "${WORKSPACE}"'
                        // Fetch the latest changes from the remote repository
                        sh 'git fetch origin main'

                        // Check if Dockerfile has changed in the latest commit on the main branch
                        def dockerfileChanged = sh(
                            script: 'git diff-tree --no-commit-id --name-only -r origin/main | grep -q Dockerfile && echo "yes" || echo "no"',
                            returnStdout: true
                        ).trim() == 'yes'
                        
                        // Check if the latest image exists in the Docker repository
                        def imageExists = sh(
                            script: "docker manifest inspect ${TAG_LATEST} > /dev/null 2>&1 && echo 'yes' || echo 'no'",
                            returnStdout: true
                        ).trim() == 'yes'
                        
                        // Decide whether to build or pull the Docker image based on conditions
                        if (dockerfileChanged || !imageExists) {
                            echo "Dockerfile changed in the latest commit or no existing image found. Building Docker image."
                            imageBuilt = true
                            try {
                                // Build the Docker image with both tags (latest and build-specific)
                                sh "docker build -t ${TAG_LATEST} -t ${TAG_BUILD} ."
                            } catch (Exception e) {
                                error "Failed to build Docker image: ${e.message}"
                            }
                        } else {
                            echo "No change in Dockerfile and latest image exists. Pulling Docker image."
                            try {
                                // Pull the latest Docker image
                                sh "docker pull ${TAG_LATEST}"
                            } catch (Exception e) {
                                error "Failed to pull Docker image: ${e.message}"
                            }
                        }
                    }
                }
            }
        }

        stage('Start Container and Copy Test Package') {
            steps {
                container('docker') {
                    script {
                        // Start the container if it's not already running
                        sh "docker run -itd --name ${CONTAINER_NAME} ${TAG_LATEST}"

                        // Copy the test package into the running container
                        sh 'docker cp ./test_cross_track_error ${CONTAINER_NAME}:/root/gem_ws/src/'
                    }
                }
            }
        }

        stage('Run Test Package') {
            steps {
                container('docker') {
                    script {
                        // Execute commands in the running container, using docker exec to ensure each runs in the right context
                        sh 'docker exec ${CONTAINER_NAME} /bin/bash -c "source /opt/ros/noetic/setup.bash && source /root/gem_ws/devel/setup.bash && rostest test_cross_track_error pure_pursuit_cte_test.test duration:=${TEST_DURATION} error_threshold:=${ERROR_THRESHOLD}"'
                    }
                }
            }
        }
        
        stage('Push Docker Image') {
            when {
                expression { imageBuilt == true && (currentBuild.result == null || currentBuild.result == 'SUCCESS') }
            }
            steps {
                container('docker') {
                    script {
                        echo "Pushing Docker image to Docker Hub: ${TAG_LATEST} and ${TAG_BUILD}"
                        
                        // Use Docker Hub token for authentication
                        withCredentials([string(credentialsId: 'dockerhub-token-id', variable: 'DOCKER_TOKEN')]) {
                            sh 'echo "$DOCKER_TOKEN" | docker login --username "pritishbisht" --password-stdin'
                        }

                        try {
                            // Push both tags to Docker Hub
                            sh "docker push $TAG_LATEST"
                            sh "docker push $TAG_BUILD"
                        } catch (Exception e) {
                            error "Failed to push Docker image: ${e.message}"
                        }
                    }
                }
            }
        }
    }

    post {
        always {
            script {  
                // Stopping and removing the container 
                sh "docker stop ${CONTAINER_NAME} || true"
                sh "docker rm ${CONTAINER_NAME} || true"
            }
        }
        success {
            echo 'Pipeline completed successfully!'
            mail to: "${params.EMAIL_RECIPIENTS}",
                subject: "Jenkins Build SUCCESS - ${env.JOB_NAME} #${env.BUILD_NUMBER}",
                body: "Good news! The build completed successfully.\n\n" +
                      "Job: ${env.JOB_NAME}\n" +
                      "Build Number: ${env.BUILD_NUMBER}\n" +
                      "Status: SUCCESS\n" +
                      "Check details at: ${env.BUILD_URL}"
        }
        failure {
            echo 'Pipeline failed!'
            mail to: "${params.EMAIL_RECIPIENTS}",
                subject: "Jenkins Build FAILURE - ${env.JOB_NAME} #${env.BUILD_NUMBER}",
                body: "The build failed.\n\n" +
                      "Job: ${env.JOB_NAME}\n" +
                      "Build Number: ${env.BUILD_NUMBER}\n" +
                      "Status: FAILURE\n" +
                      "Error: Cross-track error exceeded acceptable limits.\n" +
                      "Check details at: ${env.BUILD_URL}"
        }
    }
}