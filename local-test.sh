# /bin/bash
# excute something from .gitlab-ci.yml locally
# pass the job to run as an argument

cmd="gitlab-runner exec docker \
	--docker-volumes /var/run/docker.sock:/var/run/docker.sock \
	--env http_proxy="http://172.17.0.1:3128" --env https_proxy="http://172.17.0.1:3128" \
	--env no_proxy="$no_proxy" \
	--env CI_COMMIT_REF_SLUG=local \
	--env CI_REGISTRY=cr-ae2-registry.de.bosch.com \
	$@"

echo $cmd
$cmd

