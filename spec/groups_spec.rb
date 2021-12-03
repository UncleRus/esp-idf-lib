# frozen_string_literal: true

require_relative "spec_helper"
require_relative "group_list"

file = File.join(File.dirname(__FILE__), "..", "groups.yml")

RSpec.describe GroupList do
  subject { GroupList.new(file) }

  it "creates new instance" do
    expect { subject }.not_to raise_error
  end

  describe "#load_file" do
    it "does not raise" do
      expect { subject.load_file }.not_to raise_error
    end
  end

  describe "#parse" do
    it "does not raise" do
      expect { subject.parse }.not_to raise_error
    end
  end

  describe "#metadata" do
    it "returns Array" do
      expect(subject.metadata).to be_a Array
    end
  end
end

RSpec.describe "Group list metadata #{file}" do
  groups = GroupList.new(file)

  groups.all.each do |group|
    describe "Group #{group}" do
      subject { group }

      it "is a Group" do
        expect(group).to be_a Group
      end

      it "has name as a key" do
        expect(group.name?).to be true
      end

      it "has non-empty name" do
        expect(group.name).not_to be_empty
      end

      it "has description as a key" do
        expect(group.description?).to be true
      end

      it "has non-empty description" do
        expect(group.description).not_to be_empty
      end

      it "is a unique group" do
        expect(groups.lookup(group.name).length).to be 1
      end
    end
  end
end
