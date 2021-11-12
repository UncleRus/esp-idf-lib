# frozen_string_literal: true

require_relative "spec_helper"

components.each do |c|
  RSpec.describe c.to_s do
    subject { c }

    it "has .eil.yml" do
      file = File.join(subject.path, ".eil.yml")
      expect(File.exist?(file)).to be true
    end

    describe "its .eil.yml file" do
      subject { c.metadata }

      it "is valid YAML" do
        expect { subject }.not_to raise_error
      end

      it "has `components` as a top-level key" do
        expect(subject.key?("components")).to be true
      end
    end
  end
end
