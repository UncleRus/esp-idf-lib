# frozen_string_literal: true

require_relative "spec_helper"
require_relative "target_list"

file = File.join(File.dirname(__FILE__), "..", "targets.yml")

RSpec.describe "Target list metadata #{file}" do
  targets = TargetList.new(file)

  targets.all.each do |target|
    describe "Target #{target}" do
      subject { Target.new(target) }

      it "has a name" do
        expect(subject.name?).to be true
      end

      it "has non-empty name" do
        expect(subject.name).not_to be_empty
      end

      it "is a unique target in the list" do
        expect(targets.lookup(subject.name).length).to be 1
      end
    end
  end
end
