#!/usr/bin/env ruby

# this ruby script captures some screenshots from a kinect depth/rgb stream
# viewer app (fakenect_regview, which has been specifically created as a
# companion to this script) and saves them to an output dir called
# registered_playback_tests/ so that they can be visually evaluated by a human.

testname = ARGV[0]
raise "usage: make_tests.rb recording_name" unless testname

test_output_dir = "registered_playback_tests"

`mkdir #{test_output_dir}` unless File.exist?(test_output_dir)

recording_name = "/tmp/#{testname}"

puts "recording a short capture into #{recording_name}"
IO.popen("record #{recording_name}") do |io|
  sleep 2
  `kill -SIGINT #{io.pid}`
end
sleep 3

puts "starting fakenect_regview to get a depth overlay from a live kinect"
IO.popen("../build/bin/fakenect_regview 2") do |io|
  sleep 2
  `import -window "libfreenect Registration viewer" #{test_output_dir}/screenshot_#{testname}_regdepth-on-rgb-overlay_live.png`
  `kill #{io.pid}`
end
puts "done."

puts "starting fakenect_regview to get a depth overlay from prerecorded test via fakenect"
ENV["LD_PRELOAD"] = "../build/lib/fakenect/libfreenect.so"
ENV["FAKENECT_PATH"] = recording_name
IO.popen("../build/bin/fakenect_regview 2") do |io|
  sleep 2
  `import -window "libfreenect Registration viewer" #{test_output_dir}/screenshot_#{testname}_regdepth-on-rgb-overlay_fakenect.png`
  `kill #{io.pid}`
end
puts "done."
sleep 3

puts "starting fakenect_regview to get an rgb image from prerecorded test via fakenect"
ENV["LD_PRELOAD"] = "../build/lib/fakenect/libfreenect.so"
ENV["FAKENECT_PATH"] = recording_name
IO.popen("../build/bin/fakenect_regview 3") do |io|
  sleep 2
  `import -window "libfreenect Registration viewer" #{test_output_dir}/screenshot_#{testname}_rgb_live.png`
  `kill #{io.pid}`
end
puts "done."
